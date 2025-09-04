#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <Eigen/Dense>
#include <vector>

/////////////////////////////////////////////////////////////////////////////////////////

// Helper-Function to construct identity matrix
std::vector<std::vector<int>> eye(int n) {
    std::vector<std::vector<int>> matrix(n, std::vector<int>(n, 0));
    for (int i = 0; i < n; ++i) {
        matrix[i][i] = 1;
    }
    return matrix;
}

// Helper-Function to create a zero matrix
std::vector<std::vector<double>> zeros(int rows, int cols) {
    return std::vector<std::vector<double>>(rows, std::vector<double>(cols, 0.0));
}


// Helper-Function Horizontal concatenation of two 2x2 matrices
std::vector<std::vector<double>> hConcat(const std::vector<std::vector<double>>& A,
                                         const std::vector<std::vector<double>>& B) {
    int rows = A.size();
    std::vector<std::vector<double>> C(rows);
    for (int i = 0; i < rows; ++i) {
        C[i].insert(C[i].end(), A[i].begin(), A[i].end());
        C[i].insert(C[i].end(), B[i].begin(), B[i].end());
    }
    return C;
}

// Helper to compute Gamma and Gamma perp
Eigen::Vector2d Gamma(double theta) {
    return Eigen::Vector2d(cos(theta), sin(theta));
}
Eigen::Vector2d GammaPerp(double theta) {
    return Eigen::Vector2d(-sin(theta), cos(theta));
}

////////////////////////////////////////////////////////////////////////////////////////

// Main model function
std::vector<double> marine_model_step(
    double m, double m0, double L,
    double theta, double theta_dot,
    const Eigen::Vector3d& Q0, const Eigen::Vector3d& Q, const Eigen::Vector3d& Qu
) {
    Eigen::Vector2d gamma = Gamma(theta);
    Eigen::Vector2d gamma_perp = GammaPerp(theta);

    Eigen::Matrix3d M;
    M.setZero();
    M(0,0) = m + m0;
    M(1,1) = m + m0;
    M(0,2) = m * L * gamma_perp(0);
    M(1,2) = m * L * gamma_perp(1);
    M(2,0) = m * L * gamma_perp(0);
    M(2,1) = m * L * gamma_perp(1);
    M(2,2) = m * L * L;

    Eigen::Vector3d rhs;
    rhs << Q0(0) + Q(0) + Qu(0) + m * L * theta_dot * theta_dot * gamma(0),
           Q0(1) + Q(1) + Qu(1) + m * L * theta_dot * theta_dot * gamma(1),
           0;

    Eigen::Vector3d q_ddot = M.inverse() * rhs;
    return {q_ddot(0), q_ddot(1), q_ddot(2)};
}

/////////////////////////////////////////////////////////////////////////////////////////

class MarineModelNode : public rclcpp::Node {
public:
    MarineModelNode()
        : rclcpp::Node("marine_model_node"),
          theta_(0.0), theta_dot_(0.0),
          F_0_(Eigen::Vector2d::Zero()), F_(Eigen::Vector2d::Zero()), F_u_(Eigen::Vector2d::Zero()),
          prev_v_0_(Eigen::Vector2d::Zero()), prev_v_(Eigen::Vector2d::Zero())
    {
        theta_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "theta_topic", 10,
            [this](std_msgs::msg::Float64::SharedPtr msg) {
                theta_ = msg->data;
            }
        );
        theta_dot_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "theta_dot_topic", 10,
            [this](std_msgs::msg::Float64::SharedPtr msg) {
                theta_dot_ = msg->data;
            }
        );
        F_0_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "F_0_topic", 10,
            [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() == 2) {
                    F_0_ << msg->data[0], msg->data[1];
                }
            }
        );
        F_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "F_topic", 10,
            [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() == 2) {
                    F_ << msg->data[0], msg->data[1];
                }
            }
        );
        F_u_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "F_u_topic", 10,
            [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() == 2) {
                    F_u_ << msg->data[0], msg->data[1];
                }
            }
        );

        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("marine_state", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MarineModelNode::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        double m = 5;
        double m0 = 15;
        double L = 10;
        double c_0 = 2.0; // Damping coefficient for F_0
        double c   = 1.0; // Damping coefficient for F
        double theta = theta_;
        double theta_dot = theta_dot_;

        // Ocean velocity (constant)
        Eigen::Vector2d v_c(0.1, 0.1); // Example: 0.1 m/s in both x and y directions

        // Use previous velocities for v_0 and v
        Eigen::Vector2d v_0 = prev_v_0_;
        Eigen::Vector2d v   = prev_v_;

        // Compute damping forces
        Eigen::Vector2d F_0 = -c_0 * (v_0 - v_c);
        Eigen::Vector2d F   = -c   * (v   - v_c);
        Eigen::Vector2d F_u = F_u_;

        Eigen::Matrix<double, 2, 3> J_0;
        J_0 << 1, 0, 0,
               0, 1, 0;
        Eigen::Vector2d gamma_perp = GammaPerp(theta);
        Eigen::Matrix<double, 2, 3> J;
        J << 1, 0, L * gamma_perp(0),
             0, 1, L * gamma_perp(1);

        Eigen::Vector3d Q0 = J_0.transpose() * F_0;
        Eigen::Vector3d Q  = J.transpose()   * F;
        Eigen::Vector3d Qu = J_0.transpose() * F_u;

        std::vector<double> new_state = marine_model_step(m, m0, L, theta, theta_dot, Q0, Q, Qu);

        // Update previous velocities for next iteration
        if (new_state.size() >= 2) {
            prev_v_0_ << new_state[0], new_state[1];
            prev_v_  << new_state[0], new_state[1];
        }

        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = new_state;
        publisher_->publish(msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr theta_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr theta_dot_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr F_0_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr F_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr F_u_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double theta_;
    double theta_dot_;
    Eigen::Vector2d F_0_;
    Eigen::Vector2d F_;
    Eigen::Vector2d F_u_;
    Eigen::Vector2d prev_v_0_;
    Eigen::Vector2d prev_v_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MarineModelNode>());
    rclcpp::shutdown();
    return 0;
}