main() {
    
    cmd="$1"
    path="$( cd "$( dirname "${BASH_SOURCE[0]}" )/" && pwd )"
    file_name="${path}/config/case${cmd}.yaml"
    echo ${file_name}
    rosparam load ${file_name}
    
    roslaunch simulator sim.launch
} 

main "$@"