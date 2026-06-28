echo "activate zephyr environment and build dir"

ZEPHYR_BASE=~/zephyrproject/zephyr
ZEPHYR_SDK=~/zephyr-sdk-1.0.1

if [ -d "$ZEPHYR_BASE" ]; then
    echo "activate virtual environment"
    source ~/zephyrproject/.venv/bin/activate

    echo "set zephyr toolchain"
    export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
    export ZEPHYR_SDK_INSTALL_DIR=$ZEPHYR_SDK

    cd "$ZEPHYR_BASE"
    source zephyr-env.sh
fi

cd - >/dev/null

mkdir -p build
cd build

echo "now run \"cmake ..\" and after \"make -j\""

