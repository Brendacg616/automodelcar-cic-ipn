BUILD_TYPE="DEBUG"

for i in "$@"
do
case $i in
 -s)
    BUILD_TYPE="RELEASE"
    shift # past argument=value
    ;; 
  *)
    #unknown option
    ;;
esac
done

catkin_make --pkg cic -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
