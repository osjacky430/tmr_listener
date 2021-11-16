#ifndef COORDINATE_MANAGER_HPP_
#define COORDINATE_MANAGER_HPP_

#include <boost/filesystem.hpp>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <string>
#include <tmr_ext_script/tmr_parameterized_object.hpp>
#include <tmr_listener_plugins/CoordinateManagerConfig.h>
#include <yaml-cpp/yaml.h>

#include "tmr_listener/tmr_listener_handle.hpp"

namespace tmr_listener_plugins {

class CoordinateManager final : public tmr_listener::ListenerHandle {
 private:
  using CoordinateManagerCfgServer    = dynamic_reconfigure::Server<CoordinateManagerConfig>;
  using CoordinateManagerCfgServerPtr = std::unique_ptr<CoordinateManagerCfgServer>;

  enum class CoordinateJob { SaveCoordinate, LoadCoordinate };

  CoordinateJob job_option_;
  std::string yaml_dir_;
  YAML::Node cfg_;
  YAML::const_iterator pt_name_;

  CoordinateManagerCfgServerPtr reconfigure_server_ = [this]() {
    auto ret_val = std::make_unique<CoordinateManagerCfgServer>();
    ret_val->setCallback([this](CoordinateManagerConfig& t_cfg, std::uint32_t /**/) {
      if (not t_cfg.yaml_path.empty() and this->yaml_dir_ != t_cfg.yaml_path) {
        this->yaml_dir_ = t_cfg.yaml_path;
        this->cfg_      = YAML::LoadFile(t_cfg.yaml_path);
        this->pt_name_  = this->cfg_.begin();
      }
    });

    return ret_val;
  }();

 public:
  void response_msg(tmr_listener::TMSTAResponse::DataMsg const& t_resp) override {
    using namespace tmr_listener;
    auto const point_coor_str = t_resp.data_.substr(t_resp.data_.find('{'));
    auto const point_coor     = parse_as<double, 6>(point_coor_str);

    for (std::size_t i = 0; i < point_coor.size(); ++i) {
      this->cfg_[this->pt_name_->first.as<std::string>()][i] = point_coor[i];
    }

    ++this->pt_name_;
  }

  tmr_listener::Decision start_task(std::string const& t_data) override {
    using namespace std::string_literals;
    if (t_data == "SaveCoordinate"s) {
      this->job_option_ = CoordinateJob::SaveCoordinate;
      return tmr_listener::Decision::Accept;
    }

    if (t_data == "LoadCoordinate"s) {
      this->job_option_ = CoordinateJob::LoadCoordinate;
      return tmr_listener::Decision::Accept;
    }

    return tmr_listener::Decision::Ignore;
  }

  tmr_listener::MessagePtr generate_cmd(tmr_listener::ListenerHandle::MessageStatus const t_status) override {
    using namespace std::string_literals;
    using namespace tmr_listener;

    if (t_status == ListenerHandle::MessageStatus::Responded) {
      if (this->cfg_.size() == 0) {
        return dummy_command_list("YAMLNotLoaded"s);
      }

      if (this->job_option_ == CoordinateJob::SaveCoordinate) {
        if (this->pt_name_ == this->cfg_.end()) {
          std::ofstream f(this->yaml_dir_);
          f << this->cfg_;

          return TMSCT << TMR_ID("finish") << ScriptExit::WithPriority(ScriptExit::Result::ScriptFail);
        }

        // local variable (to this program), project variable (to robot arm)
        Variable<std::string> point_name{"var_save_point"};
        return TMSCT << TMR_ID("SavePointFromProject") << (point_name = this->pt_name_->first.as<std::string>())
                     << ScriptExit();
      }

      auto ret_val = TMSCT << TMR_ID("LoadPointToProject");
      for (auto const& pt : this->cfg_) {
        auto const coor_vec = pt.second.as<std::vector<float>>();
        assert(coor_vec.size() == 6);

        ret_val << (Point[pt.first.as<std::string>()].Value = std::array<float, 6>{
                      coor_vec[0], coor_vec[1], coor_vec[2], coor_vec[3], coor_vec[4], coor_vec[5]});
      }

      return ret_val << ScriptExit();
    }

    return dummy_command_list("waiting"s);
  }
};

}  // namespace tmr_listener_plugins

#endif