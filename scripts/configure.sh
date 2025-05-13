#!/bin/bash
# Bash script to push configuration files to Android device via adb

# Parse flags
while getopts "c:s:" opt; do
  case ${opt} in
    c ) config="$OPTARG" ;;
    s ) season="$OPTARG" ;;
    * ) echo "Usage: $0 -c <config> -s <season>"; exit 1 ;;
  esac
done

# Validate arguments
if [ -z "$config" ]; then
  echo -e "\e[31mError: No configuration specified. Use -c to specify the configuration.\e[0m"
  exit 1
fi

if [ -z "$season" ]; then
  echo -e "\e[31mError: No season specified. Use -s to specify the season.\e[0m"
  exit 1
fi

# Resolve script directory
script_path="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Installing configuration $config from season $season"

# Push hardware map configuration file
echo "--> Pushing hardware map configuration file"
conf_hwmap_file_path="$script_path/../$season/conf/hwmap/$config.xml"
remote_path="/sdcard/FIRST/$config.xml"

adb push "$conf_hwmap_file_path" "$remote_path"
if [ $? -eq 0 ]; then
  echo -e "\e[32mConfiguration file pushed successfully.\e[0m"
else
  echo -e "\e[31mFailed to push configuration file.\e[0m"
fi

# Push robot configuration file
echo "--> Pushing robot configuration file"
conf_robot_file_path="$script_path/../$season/conf/robot/$config.json"
remote_path="/sdcard/FIRST/$config.json"

adb push "$conf_robot_file_path" "$remote_path"
if [ $? -eq 0 ]; then
  echo -e "\e[32mConfiguration file pushed successfully.\e[0m"
else
  echo -e "\e[31mFailed to push configuration file.\e[0m"
fi