#include "rclcpp/rclcpp.hpp"

class CounterNode: public rclcpp::Node {
    public:
        CounterNode(): Node("cpp_test"), counter(0){
            RCLCPP_INFO(this->get_logger(), "Hello Cpp Node");
            timer = this->create_wall_timer(std::chrono::seconds(1), 
                std::bind(&CounterNode::timerCallback, this));

        }
        
    private:
        rclcpp::TimerBase::SharedPtr timer;
        int counter;

        void timerCallback(){
            counter ++;
            RCLCPP_INFO(this -> get_logger(), "Counter: %d", counter);
        }

};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}