
cd ../robot

DAZEL_RC_FILE=".dazelrc_20.04" DAZEL_BAZEL_RC_FILE=".dazel_bazelrc" dazel build --config=python-3.8 //experimental/beacon_sim:beacon_sim_wheel
WHEEL_NAME_X86_64=`cat bazel-bin/experimental/beacon_sim/beacon_sim_wheel.name`
cp bazel-bin/experimental/beacon_sim/$WHEEL_NAME_X86_64 ../robot-spot

bazel build --config=python-3.8 --platforms=//toolchain:gcc_aarch64 //experimental/beacon_sim:beacon_sim_wheel
WHEEL_NAME_AARCH64=`cat bazel-bin/experimental/beacon_sim/beacon_sim_wheel.name`
cp bazel-bin/experimental/beacon_sim/$WHEEL_NAME_AARCH64 ../robot-spot

cd ../robot-spot
chmod 777 $WHEEL_NAME_X86_64
chmod 777 $WHEEL_NAME_AARCH64
