#!/usr/bin/env bash
set -e
set -x

# git diff --name-status origin/release3-staging | grep "^A" | less

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"

cd $DIR

BUILD_DIR=/data/openpilot
SOURCE_DIR="$(git rev-parse --show-toplevel)"

FILES_SRC="release/files_tici"
RELEASE_BRANCH="release3"


# set git identity
source /data/identity.sh

echo "[-] Setting up repo T=$SECONDS"
rm -rf $BUILD_DIR
mkdir -p $BUILD_DIR
cd $BUILD_DIR
git init
git remote add origin https://github.com/tc1979/openpilot.git
git checkout --orphan $SOURCE_BRANCH

# do the files copy
echo "[-] copying files T=$SECONDS"
cd $SOURCE_DIR
cp -pR --parents $(./release/release_files.py) $BUILD_DIR/

# in the directory
cd $BUILD_DIR

rm -f panda/board/obj/panda.bin.signed
rm -f panda/board/obj/panda_h7.bin.signed

VERSION=$(cat common/version.h | awk -F[\"-]  '{print $2}')
echo "#define COMMA_VERSION \"$VERSION-release\"" > common/version.h

echo "[-] committing version $VERSION T=$SECONDS"
git add -f .
git commit -a -m "TOP v$VERSION release"

# Build
export PYTHONPATH="$BUILD_DIR"
scons -j$(nproc) --minimal

#if [ -z "$PANDA_DEBUG_BUILD" ]; then
#  # release panda fw
#  CERT=/data/pandaextra/certs/release RELEASE=1 scons -j$(nproc) panda/
#else
#  # build with ALLOW_DEBUG=1 to enable features like experimental longitudinal
#  scons -j$(nproc) panda/
#fi
scons -j$(nproc) panda/

# Ensure no submodules in release
if test "$(git submodule--helper list | wc -l)" -gt "0"; then
  echo "submodules found:"
  git submodule--helper list
  exit 1
fi
git submodule status

# Cleanup
find . -name '*.a' -delete
find . -name '*.o' -delete
find . -name '*.os' -delete
find . -name '*.pyc' -delete
find . -name 'moc_*' -delete
find . -name '__pycache__' -delete
rm -rf .sconsign.dblite Jenkinsfile release/
rm selfdrive/modeld/models/driving_vision.onnx
rm selfdrive/modeld/models/driving_policy.onnx

find third_party/ -name '*x86*' -exec rm -r {} +
find third_party/ -name '*Darwin*' -exec rm -r {} +


# Restore third_party
git checkout third_party/

# Mark as prebuilt release
touch prebuilt

# dragonpilot customized
find . -name '*.cc' -delete
find selfdrive/ui/ -name '*.h' -delete
rm -fr selfdrive/modeld/models/dmonitoring_model*
find . -type d -name "tests" -exec rm -rf {} +
find . -type d -name 'x86_64' -exec rm -rf {} +
find . -type d -name 'Darwin' -exec rm -rf {} +
find . -name 'amd_gpu.py' -delete # 3.8M
rm -fr system/loggerd/bootlog
cp -fr selfdrive/ui/qt/text_larch64 selfdrive/ui/qt/_text
rm -fr selfdrive/ui/qt/text_larch64 # 3.7M
cp -fr selfdrive/ui/qt/spinner_larch64 selfdrive/ui/qt/_spinner
rm -fr selfdrive/ui/qt/spinner_larch64 # 3.7M
rm -fr tinygrad_repo/docs/tinygrad_intro.pdf # 1.9M
rm -fr cereal/gen/cpp/log.capnp.h # 2.5M
rm -fr tinygrad_repo/extra/hip_gpu_driver/gc_10_3_0_offset.h # 1.4 M
rm -fr tinygrad_repo/extra/accel/tpu/logs/tpu_driver.t1v-n-852cd0d5-w-0.taylor.log.INFO.20210619-062914.26926.gz # 1.3 M

# Add built files to git
git add -f .
git commit --amend -m "TOP v$VERSION"

# Run tests
#cd $BUILD_DIR
#RELEASE=1 pytest -n0 -s selfdrive/test/test_onroad.py
##pytest selfdrive/car/tests/test_car_interfaces.py
git branch -m $RELEASE_BRANCH
#if [ ! -z "$RELEASE_BRANCH" ]; then
#  echo "[-] pushing release T=$SECONDS"
#  git push -f origin $RELEASE_BRANCH:$RELEASE_BRANCH
#fi

echo "[-] done T=$SECONDS"
