cmake_minimum_required(VERSION 3.16)
project(reachy_mini)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Reachy Mini settings of the audio nodes, the robot blackboard, speech
# recognition and synthesis: launch_pal merges them over the nodes' own
# defaults when their launch files run
ament_index_register_resource(pal_configuration.mic_frontend
  CONTENT config/mic_frontend/10-reachy-mini.yml)
ament_index_register_resource(pal_configuration.pw_audio_play
  CONTENT config/pw_audio_play/10-reachy-mini.yml)
ament_index_register_resource(pal_configuration.robot_blackboard
  CONTENT config/robot_blackboard/10-reachy-mini.yml)
ament_index_register_resource(pal_configuration.asr_whistle
  CONTENT config/asr_whistle/10-reachy-mini.yml)
ament_index_register_resource(pal_configuration.tts_pockettts
  CONTENT config/tts_pockettts/10-reachy-mini.yml)

ament_auto_package(INSTALL_TO_SHARE launch config systemd)
