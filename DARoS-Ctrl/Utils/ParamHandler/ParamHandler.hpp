#ifndef PARAMETER_HANDLER
#define PARAMETER_HANDLER

#include <daros_yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <cppTypes.h>

class ParamHandler {
public:
  ParamHandler(const std::string &file_name);

  virtual ~ParamHandler();

  bool getString(const std::string &key, std::string &str_value);
  bool getString(const std::string &category, const std::string &key, std::string & str_value);

  template<typename T>
  bool getVector(const std::string &key, std::vector<T> &vec_value) {
    try {
      vec_value = config_[key].as<std::vector<T> >();
    } catch (std::exception &e) {
      return false;
    }
    return true;
  }

  template<typename T>
  bool getEigenVec(const std::string & key, DVec<T> & vec_value) {
    try {
      std::vector<T> vec = config_[key].as<std::vector<T> >();
      vec_value = Eigen::Map<DVec<T> >(vec.data(), vec.size());
    } catch (std::exception &e) {
      return false;
    }
    return true;
  }

  template<typename T>
  bool getVec3Sequence(const std::string & key, std::vector<Vec3<T>> & vec3_seq) {
    try {
      std::vector< std::vector<T> > vec3_seq_value = config_[key].as< std::vector< std::vector<T> > >();
      vec3_seq.resize(vec3_seq_value.size());
      for (size_t i = 0; i < vec3_seq_value.size(); i++) {
        // vec3_seq[i] = Eigen::Map<Vec3<T> >(vec3_seq_value[i].data(), vec3_seq_value[i].size());
        vec3_seq[i] = Eigen::Map<Vec3<T> >(vec3_seq_value[i].data(), 3);
      }
    } catch (std::exception &e) {
      return false;
    }
    return true;
  }


  template<typename T>
  bool getVector(const std::string &category, const std::string &key, 
  std::vector<T> &vec_value) {
    try {
      vec_value = config_[category][key].as<std::vector<T>>();
    } catch (std::exception &e) {
      return false;
    }
    return true;
  }

  template<typename T>
  bool get2DArray(const std::string &category, const std::string &key, 
  std::vector<std::vector<T> > &vec_value) {
    try {
      vec_value = config_[category][key].as<std::vector<std::vector<T> > >();
    } catch (std::exception &e) {
      return false;
    }
    return true;
  }

  template<typename T>
  bool get2DArray(const std::string &key, std::vector<std::vector<T> > &vec_value) {
    try {
      vec_value = config_[key].as<std::vector<std::vector<T> > >();
    } catch (std::exception &e) {
      return false;
    }
    return true;
  }

  template<typename T>
  bool getValue(const std::string &key, T &T_value) {
    try {
      T_value = config_[key].as<T>();
    } catch (std::exception &e) {
      return false;
    }
    return true;
  }

  template<typename T>
  bool getValue(const std::string &category, const std::string &key, T &T_value) {
    try {
      T_value = config_[category][key].as<T>();
      return true;
    } catch (std::exception &e) {
      return false;
    }
    return true;
  }

  bool getBoolean(const std::string & category, const std::string &key, bool &bool_value){
    try {
      bool_value = config_[category][key].as<bool>();
      return true;
    } catch (std::exception &e) {
      return false;
    }
    return true;
  }


  std::vector<std::string> getKeys() {
    std::vector<std::string> v;
    v.reserve(config_.size());
    for(auto it = config_.begin(); it != config_.end(); it++) {
      v.push_back(it->first.as<std::string>());
    }
    return v;
  }


  bool getBoolean(const std::string &key, bool &bool_value);

  bool getInteger(const std::string &key, int &int_value);

  bool fileOpenedSuccessfully() {
    return fileLoaded;
  }

protected:
  daros_YAML::Node config_;

private:
  bool fileLoaded = false;
};

#endif
