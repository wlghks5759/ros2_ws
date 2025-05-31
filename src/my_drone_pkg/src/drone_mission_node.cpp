#include <rclcpp/rclcpp.hpp>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/mission/mission.h> // mission_raw 대신 사용 가능

#include <sensor_msgs/msg/nav_sat_fix.hpp> // 현재 GPS 읽기용 (선택적)
#include <std_srvs/srv/trigger.hpp> // 비상 상황 서비스용

#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <future>

using namespace mavsdk;
using namespace std::chrono_literals; // for s, ms, etc.
using std::this_thread::sleep_for;

// 지구 반경 (미터)
const double EARTH_RADIUS_M = 6371000.0;

// 도 단위를 라디안으로 변환
double to_radians(double degrees) {
    return degrees * M_PI / 180.0;
}

// 두 GPS 좌표 간의 거리 계산 (Haversine formula)
double calculate_distance_haversine(double lat1, double lon1, double lat2, double lon2) {
    double dLat = to_radians(lat2 - lat1);
    double dLon = to_radians(lon2 - lon1);

    lat1 = to_radians(lat1);
    lat2 = to_radians(lat2);

    double a = sin(dLat / 2) * sin(dLat / 2) +
               sin(dLon / 2) * sin(dLon / 2) * cos(lat1) * cos(lat2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return EARTH_RADIUS_M * c;
}

// 두 GPS 좌표 간의 방위각 계산 (초기 방위각)
double calculate_bearing(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg) {


    double dLon_deg = lon2_deg - lon1_deg;
    double dLon_rad = to_radians(dLon_deg);

    double lat1_rad = to_radians(lat1_deg);
    double lat2_rad = to_radians(lat2_deg); // 목표 위도를 라디안으로 변환


    // 수정된 계산 (일반적인 방위각 공식)
    // https://www.movable-type.co.uk/scripts/latlong.html
    double y_calc = sin(dLon_rad) * cos(lat2_rad);
    double x_calc = cos(lat1_rad) * sin(lat2_rad) -
                    sin(lat1_rad) * cos(lat2_rad) * cos(dLon_rad);


    double bearing_rad_calc = atan2(y_calc, x_calc);
    double bearing_deg_calc = fmod((bearing_rad_calc * 180.0 / M_PI + 360.0), 360.0); // 0-360도 범위로 정규화


    return bearing_deg_calc;
}


class DroneMissionNode : public rclcpp::Node {
public:
    DroneMissionNode() : Node("drone_mission_node") {
        // 파라미터 선언
        this->declare_parameter<std::string>("connection_url", "udp://:14540");
        this->declare_parameter<double>("target_latitude", 0.0);   // 기본값
        this->declare_parameter<double>("target_longitude", 0.0);  // 기본값
        this->declare_parameter<float>("target_relative_altitude_m", 10.0f); // 기본 이륙 및 목표 고도
        this->declare_parameter<float>("takeoff_altitude_m", 5.0f);
        this->declare_parameter<float>("arrival_threshold_m", 5.0f); // 목표 도달 반경
        this->declare_parameter<float>("max_speed_m_s", 2.0f); // 최대 수평 속도
        this->declare_parameter<float>("max_vertical_speed_m_s", 1.0f); // 최대 수직 속도


        // 파라미터 가져오기
        connection_url_ = this->get_parameter("connection_url").as_string();
        target_latitude_ = this->get_parameter("target_latitude").as_double();
        target_longitude_ = this->get_parameter("target_longitude").as_double();
        target_relative_altitude_m_ = this->get_parameter("target_relative_altitude_m").as_double();
        takeoff_altitude_m_ = this->get_parameter("takeoff_altitude_m").as_double();
        arrival_threshold_m_ = this->get_parameter("arrival_threshold_m").as_double();
        max_speed_m_s_ = this->get_parameter("max_speed_m_s").as_double();
        max_vertical_speed_m_s_ = this->get_parameter("max_vertical_speed_m_s").as_double();


        RCLCPP_INFO(this->get_logger(), "Connecting to drone at: %s", connection_url_.c_str());
        RCLCPP_INFO(this->get_logger(), "Target: Lat=%.7f, Lon=%.7f, Alt=%.2fm",
                    target_latitude_, target_longitude_, target_relative_altitude_m_);
        RCLCPP_INFO(this->get_logger(), "Takeoff Altitude: %.2fm", takeoff_altitude_m_);

        // 비상 정지 서비스 서버 생성
        emergency_service_ = this->create_service<std_srvs::srv::Trigger>(
            "emergency_stop",
            std::bind(&DroneMissionNode::handle_emergency_stop, this, std::placeholders::_1, std::placeholders::_2));

        // MAVSDK 초기화 및 메인 로직 실행 (별도 스레드에서 실행하여 ROS 콜백 방해 안함)
        mission_thread_ = std::thread(&DroneMissionNode::run_mission, this);
    }

    ~DroneMissionNode() {
        if (mission_thread_.joinable()) {
            // MAVSDK 플러그인들을 여기서 정리하거나, run_mission 끝에서 정리
            // Offboard 모드가 활성화된 상태로 종료되지 않도록 주의
            if (offboard_ && system_ && offboard_->is_active()) {
                RCLCPP_INFO(this->get_logger(), "Stopping offboard mode before exit...");
                auto stop_result = offboard_->stop();
                 if (stop_result != Offboard::Result::Success) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to stop offboard");
                }
            }
            // MAVSDK 객체 소멸 시 자동으로 연결 해제 및 정리됨
            mission_thread_.join();
        }
        RCLCPP_INFO(this->get_logger(), "Drone mission node shutdown.");
    }

private:
    void run_mission() {
        mavsdk_ = std::make_shared<Mavsdk>(Mavsdk::Configuration{ComponentType::GroundStation});

        ConnectionResult connection_result = mavsdk_->add_any_connection(connection_url_);
        if (connection_result != ConnectionResult::Success) {
            RCLCPP_ERROR(this->get_logger(), "Connection failed");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "MAVSDK connection established.");

        // 시스템 발견 대기
        RCLCPP_INFO(this->get_logger(), "Waiting for system to connect...");
        auto discovered_system_promise = std::promise<std::shared_ptr<System>>();
        auto discovered_system_future = discovered_system_promise.get_future();

        mavsdk_->subscribe_on_new_system([this, &discovered_system_promise]() {
            const auto systems = mavsdk_->systems();
            if (!systems.empty()) {
                RCLCPP_INFO(this->get_logger(), "System discovered!");
                discovered_system_promise.set_value(systems.front());
                mavsdk_->subscribe_on_new_system(nullptr); // Unsubscribe
            }
        });

        if (discovered_system_future.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
             RCLCPP_ERROR(this->get_logger(), "No system discovered in 10 seconds.");
             rclcpp::shutdown();
             return;
        }
        system_ = discovered_system_future.get();


        action_ = std::make_shared<Action>(system_);
        telemetry_ = std::make_shared<Telemetry>(system_);
        offboard_ = std::make_shared<Offboard>(system_);

        // 텔레메트리 구독 (홈 위치, 현재 위치 등)
        telemetry_->subscribe_health_all_ok([this](bool all_ok){
            if(!initial_health_ok_ && all_ok) {
                RCLCPP_INFO(this->get_logger(), "System health is OK.");
                initial_health_ok_ = true;
            } else if (initial_health_ok_ && !all_ok) {
                RCLCPP_WARN(this->get_logger(), "System health problem detected!");
            }
        });

        telemetry_->subscribe_home([this](Telemetry::Position home_pos){
            if(!home_position_set_){
                home_latitude_ = home_pos.latitude_deg;
                home_longitude_ = home_pos.longitude_deg;
                home_absolute_altitude_m_ = home_pos.absolute_altitude_m;
                home_relative_altitude_m_ = home_pos.relative_altitude_m; // Should be 0 at home
                home_position_set_ = true;
                RCLCPP_INFO(this->get_logger(), "Home position set: Lat=%.7f, Lon=%.7f, AbsAlt=%.2fm",
                            home_latitude_, home_longitude_, home_absolute_altitude_m_);
            }
        });
        
        // 현재 GPS 위치 구독
        telemetry_->subscribe_position([this](Telemetry::Position position) {
            std::lock_guard<std::mutex> lock(position_mutex_);
            current_latitude_ = position.latitude_deg;
            current_longitude_ = position.longitude_deg;
            current_relative_altitude_m_ = position.relative_altitude_m;
            current_absolute_altitude_m_ = position.absolute_altitude_m;
        });


        // 1. 이륙 전 상태 확인 (GPS Lock, Home 설정 등)
        RCLCPP_INFO(this->get_logger(), "Waiting for vehicle to have a GPS fix and home position...");
        while (!telemetry_->health_all_ok() || !home_position_set_) {
            if (emergency_triggered_) { RCLCPP_WARN(this->get_logger(), "Emergency triggered during pre-flight checks. Aborting."); return; }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for GPS fix and home position...");
            sleep_for(1s);
        }
        RCLCPP_INFO(this->get_logger(), "GPS fix and home position acquired.");


        // 2. Arm
        RCLCPP_INFO(this->get_logger(), "Arming...");
        const Action::Result arm_result = action_->arm();
        if (arm_result != Action::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Arming failed");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Armed.");

        // 3. Takeoff
        RCLCPP_INFO(this->get_logger(), "Setting takeoff altitude to %.2f m", takeoff_altitude_m_);
        action_->set_takeoff_altitude(takeoff_altitude_m_);
        RCLCPP_INFO(this->get_logger(), "Taking off...");
        const Action::Result takeoff_result = action_->takeoff();
        if (takeoff_result != Action::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Takeoff failed");
            rclcpp::shutdown();
            return;
        }

        // 이륙 완료 대기
        while (true) {
            if (emergency_triggered_) { RCLCPP_WARN(this->get_logger(), "Emergency triggered during takeoff. Attempting to land."); break; }
            float rel_alt;
            {
                std::lock_guard<std::mutex> lock(position_mutex_);
                rel_alt = current_relative_altitude_m_;
            }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Current altitude: %.2f m", rel_alt);
            if (rel_alt >= takeoff_altitude_m_ * 0.95f) { // 95% 도달 시
                RCLCPP_INFO(this->get_logger(), "Takeoff complete. Reached altitude: %.2f m", rel_alt);
                break;
            }
            sleep_for(500ms);
        }
        if (emergency_triggered_) {
            // 비상 착륙 코드
            RCLCPP_INFO(this->get_logger(), "Emergency triggered, initiating landing sequence");
            // 착륙 관련 코드
        } else {
            // 정상 코드
            Offboard::VelocityNedYaw initial_setpoint{};
            offboard_->set_velocity_ned(initial_setpoint);
            Offboard::Result offboard_start_result = offboard_->start();
            // 나머지 코드
        }


        // 4. Offboard 모드로 목표 지점 이동
        RCLCPP_INFO(this->get_logger(), "Preparing for Offboard mode to target...");

        // Offboard 모드 시작 전 초기 Setpoint 전송 필수! (현재 위치에서 호버링)
        Offboard::VelocityNedYaw initial_setpoint{}; // 0,0,0,0
        initial_setpoint.north_m_s = 0.0f;
        initial_setpoint.east_m_s = 0.0f;
        initial_setpoint.down_m_s = 0.0f;
        initial_setpoint.yaw_deg = telemetry_->attitude_euler().yaw_deg; // 현재 yaw 유지
        offboard_->set_velocity_ned(initial_setpoint);
        RCLCPP_INFO(this->get_logger(), "Initial setpoint sent (hover).");
        sleep_for(100ms); // PX4가 setpoint를 수신할 시간

        Offboard::Result offboard_start_result = offboard_->start();
        if (offboard_start_result != Offboard::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Offboard start failed");
            goto land_sequence; // 실패 시 착륙 시도
        }
        RCLCPP_INFO(this->get_logger(), "Offboard mode started.");

        // 목표 지점까지 이동
        RCLCPP_INFO(this->get_logger(), "Moving to target: Lat=%.7f, Lon=%.7f, Alt=%.2fm",
                    target_latitude_, target_longitude_, target_relative_altitude_m_);

        while (rclcpp::ok() && !emergency_triggered_) {
            double current_lat_local, current_lon_local;
            float current_rel_alt_local;
            {
                std::lock_guard<std::mutex> lock(position_mutex_);
                current_lat_local = current_latitude_;
                current_lon_local = current_longitude_;
                current_rel_alt_local = current_relative_altitude_m_;
            }

            double distance_to_target = calculate_distance_haversine(
                current_lat_local, current_lon_local,
                target_latitude_, target_longitude_);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Distance to target: %.2f m, Current Alt: %.2f m",
                                 distance_to_target, current_rel_alt_local);

            if (distance_to_target < arrival_threshold_m_ &&
                std::abs(current_rel_alt_local - target_relative_altitude_m_) < 1.0f) { // 고도 오차 1m 이내
                RCLCPP_INFO(this->get_logger(), "Target reached!");
                break;
            }

            if (!offboard_->is_active()){
                RCLCPP_WARN(this->get_logger(), "Offboard mode was deactivated externally!");
                // 재시도 또는 안전 조치
                Offboard::Result re_offboard_start_result = offboard_->start();
                if (re_offboard_start_result != Offboard::Result::Success) {
                    RCLCPP_ERROR(this->get_logger(), "Offboard re-start failed");
                    goto land_sequence;
                }
                RCLCPP_INFO(this->get_logger(), "Offboard mode re-started.");
            }


            // 목표 방향으로의 속도 벡터 계산
            double bearing_rad = to_radians(calculate_bearing(current_lat_local, current_lon_local, target_latitude_, target_longitude_));
            
            // 수평 속도 계산 (거리에 비례, 최대 속도 제한)
            // 간단한 P 제어기처럼 작동.
            float p_gain_horizontal = 0.2f; // 또는 0.1f 등 더 작은 값으로 시작
            float desired_speed = std::min(max_speed_m_s_, (float)distance_to_target * p_gain_horizontal);

            float north_vel = desired_speed * cos(bearing_rad);
            float east_vel = desired_speed * sin(bearing_rad);

            // 수직 속도 계산 (고도 차이에 비례, 최대 속도 제한)
            float altitude_error = target_relative_altitude_m_ - current_rel_alt_local;
            float down_vel = -1.0f * std::max(-max_vertical_speed_m_s_, std::min(max_vertical_speed_m_s_, altitude_error * 0.5f)); // 위로 갈때 음수, 아래로 갈때 양수 (P 게인 0.5)

            // Yaw는 목표 방향을 보도록 설정 (또는 현재 yaw 유지)
            // float target_yaw_deg = calculate_bearing(current_lat_local, current_lon_local, target_latitude_, target_longitude_);
            float target_yaw_deg = calculate_bearing(current_lat_local, current_lon_local, target_latitude_, target_longitude_);

            Offboard::VelocityNedYaw speed_setpoint{};
            speed_setpoint.north_m_s = north_vel;
            speed_setpoint.east_m_s = east_vel;
            speed_setpoint.down_m_s = down_vel;
            speed_setpoint.yaw_deg = target_yaw_deg;

                    // >>>>>>>>>>>> 여기에 로그 추가 <<<<<<<<<<<<
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, // 500ms (0.5초) 간격으로 로그 출력
                "Setpoint: N:%.2f E:%.2f D:%.2f | Bearing:%.1f TargetYaw:%.1f | Dist:%.2f AltErr:%.2f",
                north_vel, east_vel, down_vel,
                calculate_bearing(current_lat_local, current_lon_local, target_latitude_, target_longitude_), // 방위각 직접 출력 (계산된 target_yaw_deg와 같아야 함)
                target_yaw_deg, // 실제 적용되는 yaw setpoint
                distance_to_target,
                altitude_error); // 고도 오차도 함께 보면 좋음

            offboard_->set_velocity_ned(speed_setpoint);

            sleep_for(100ms); // MAVSDK는 내부적으로 20Hz로 전송하지만, 제어 루프 주기 조절
        }
        if (emergency_triggered_) { RCLCPP_WARN(this->get_logger(), "Emergency triggered during flight to target."); }

        // Offboard 모드 중지
        RCLCPP_INFO(this->get_logger(), "Stopping Offboard mode...");
        {
            Offboard::Result offboard_stop_result = offboard_->stop();
            if (offboard_stop_result != Offboard::Result::Success) {
                RCLCPP_ERROR(this->get_logger(), "Offboard stop failed");
                // 계속 진행하여 착륙 시도
            } else {
                RCLCPP_INFO(this->get_logger(), "Offboard mode stopped.");
            }
        }


    land_sequence: // 착륙 시퀀스 시작점
        if (emergency_triggered_) {
            RCLCPP_WARN(this->get_logger(), "Emergency Land initiated!");
            // 비상 상황 시에는 Action의 land() 보다 더 강력한 모드 변경이 필요할 수 있음 (예: RTL, Emergency Land Mode)
            // 여기서는 일단 일반적인 land() 사용
        }

        // 5. Landing
        RCLCPP_INFO(this->get_logger(), "Landing...");
        const Action::Result land_result = action_->land();
        if (land_result != Action::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Landing failed");
            // 실패해도 일단 계속 진행하여 종료 시도
        }

        // 착륙 완료 대기
        while (telemetry_->in_air()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for landing to complete...");
            sleep_for(1s);
             if (emergency_triggered_ && !telemetry_->in_air()){ // 이미 착륙했는데 비상 플래그가 켜진 경우
                break;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Landed.");

        // (선택적) Disarm
        RCLCPP_INFO(this->get_logger(), "Disarming...");
        const Action::Result disarm_result = action_->disarm();
         if (disarm_result != Action::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Disarming failed");
        } else {
            RCLCPP_INFO(this->get_logger(), "Disarmed.");
        }


        RCLCPP_INFO(this->get_logger(), "Mission complete.");
        rclcpp::shutdown(); // ROS 노드 종료
    }

    void handle_emergency_stop(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request; // 사용 안함
        RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP SERVICE CALLED!");
        emergency_triggered_ = true;
        response->success = true;
        response->message = "Emergency stop triggered. Attempting to land.";

        // 비상 상황 시 Offboard 모드 즉시 중단 시도
        if (offboard_ && offboard_->is_active()) {
            RCLCPP_INFO(this->get_logger(), "Emergency: Stopping offboard mode immediately.");
            // 비동기 호출로 응답 지연 방지 가능하지만, 여기서는 동기적으로 처리
            auto stop_result = offboard_->stop();
            if (stop_result != Offboard::Result::Success) {
                RCLCPP_ERROR(this->get_logger(), "Emergency: Failed to stop offboard");
            }
        }
        // run_mission() 루프에서 emergency_triggered_ 플래그를 확인하고 land_sequence로 점프함.
    }


    std::string connection_url_;
    double target_latitude_;
    double target_longitude_;
    float target_relative_altitude_m_;
    float takeoff_altitude_m_;
    float arrival_threshold_m_;
    float max_speed_m_s_;
    float max_vertical_speed_m_s_;


    std::shared_ptr<Mavsdk> mavsdk_;
    std::shared_ptr<System> system_;
    std::shared_ptr<Action> action_;
    std::shared_ptr<Telemetry> telemetry_;
    std::shared_ptr<Offboard> offboard_;

    std::thread mission_thread_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_service_;
    std::atomic<bool> emergency_triggered_{false};

    // 텔레메트리 데이터용 멤버 변수 및 뮤텍스
    std::mutex position_mutex_;
    double current_latitude_{0.0};
    double current_longitude_{0.0};
    float current_relative_altitude_m_{0.0f};
    float current_absolute_altitude_m_{0.0f}; // 추가
    
    bool home_position_set_ = false;
    double home_latitude_{0.0};
    double home_longitude_{0.0};
    float home_absolute_altitude_m_{0.0f};
    float home_relative_altitude_m_{0.0f}; // 추가

    bool initial_health_ok_ = false;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneMissionNode>();
    rclcpp::spin(node); // spin은 서비스 콜백 등을 처리하기 위해 필요
    // rclcpp::shutdown()은 run_mission 내부 또는 소멸자에서 호출될 수 있음
    // 메인 스레드는 spin()에서 블로킹되고, mission_thread_가 작업을 수행.
    // mission_thread_가 rclcpp::shutdown()을 호출하면 spin()도 리턴함.
    return 0;
}
