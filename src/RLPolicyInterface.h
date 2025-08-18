#pragma once

#include <Eigen/Dense>
#include <string>
#include <memory>

// #include <onnxruntime_cxx_api.h>
#include "onnxruntime/include/onnxruntime_cxx_api.h"

/**
 * @brief Interface for RL policy inference
 * 
 * This class abstracts the RL policy implementation, allowing you to use
 * different backends (PyTorch, ONNX, TensorFlow, etc.) without changing
 * the controller code. (currently only ONNX supported)
 */
class RLPolicyInterface
{
public:
  /**
   * @brief Construct with a policy file path
   * @param policyPath Path to the policy file
   */
  RLPolicyInterface(const std::string & policyPath);
  
  /**
   * @brief Construct dummy policy (for testing)
   */
  RLPolicyInterface();
  
  /**
   * @brief Destructor
   */
  ~RLPolicyInterface();
  
  /**
   * @brief Run inference on the policy
   * @param observation Input observation vector (size 40)
   * @return Action vector (size 19)
   */
  Eigen::VectorXd predict(const Eigen::VectorXd & observation);
  
  /**
   * @brief Check if policy is loaded successfully
   * @return true if policy is ready for inference
   */
  bool isLoaded() const { return isLoaded_; }
  
  /**
   * @brief Get the expected observation size
   * @return Expected observation dimension
   */
  int getObservationSize() const { return 40; }
  
  /**
   * @brief Get the action size
   * @return Action dimension  
   */
  int getActionSize() const { return 19; }

private:
  bool isLoaded_;
  std::string policyPath_;
  
  std::unique_ptr<Ort::Session> onnxSession_;
  std::unique_ptr<Ort::Env> onnxEnv_;
  std::unique_ptr<Ort::MemoryInfo> memoryInfo_;
  std::vector<std::string> inputNames_;
  std::vector<std::string> outputNames_;
  std::vector<const char*> inputNamePtrs_;
  std::vector<const char*> outputNamePtrs_;
  std::vector<int64_t> inputShape_;
  std::vector<int64_t> outputShape_;
  
  void loadPolicy(const std::string & path);
  
  Eigen::VectorXd runOnnxInference(const Eigen::VectorXd & observation);
}; 