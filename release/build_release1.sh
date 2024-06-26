#!/usr/bin/bash
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
source $DIR/identity.sh
export GIT_SSH_COMMAND="ssh -i /data/gitkey"

echo "[-] Setting up repo T=$SECONDS"
rm -rf $BUILD_DIR
mkdir -p $BUILD_DIR
cd $BUILD_DIR
git init
source /data/identity.sh
git remote add origin git@github.com:tc1979/openpilot.git
git checkout --orphan $RELEASE_BRANCH

# do the files copy
echo "[-] copying files T=$SECONDS"
cd $SOURCE_DIR
cp -pR --parents $(./release/release_files.py; cat release/files_*) $BUILD_DIR/

# in the directory
cd $BUILD_DIR

rm -f panda/board/obj/panda.bin.signed
rm -f panda/board/obj/panda_h7.bin.signed

VERSION=$(date '+%Y.%m.%d')
echo "#define COMMA_VERSION \"$VERSION-release\"" > common/version.h

echo "[-] committing version $VERSION T=$SECONDS"
git add -f .
git commit -a -m "T.O.P v$VERSION release"

# ACADOS
wget https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux -P /data/openpilot/third_party/acados/x86_64
strip -s /data/openpilot/third_party/acados/x86_64/t_renderer-v0.0.34-linux -o /data/openpilot/third_party/acados/x86_64/t_renderer

chmod +x /data/openpilot/third_party/acados/x86_64/t_renderer

# Build
export PYTHONPATH="$BUILD_DIR"
scons -j$(nproc) --minimal

# release panda fw
CERT=panda/certs/release DEBUG=1 scons -j$(nproc) panda/

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
find . -name '*.cc' -delete
find . -name '__pycache__' -delete
find selfdrive/ui/ -name '*.h' -delete
rm -rf .sconsign.dblite Jenkinsfile release/
rm selfdrive/modeld/models/supercombo.onnx

find third_party/ -name '*x86*' -exec rm -r {} +
find third_party/ -name '*Darwin*' -exec rm -r {} +


# Restore third_party
git checkout third_party/

# Mark as prebuilt release
touch prebuilt

# include source commit hash and build date in commit
GIT_HASH=$(git --git-dir=$SOURCE_DIR/.git rev-parse HEAD)
DATETIME=$(date '+%Y-%m-%dT%H:%M:%S')
TOP_VERSION=$(cat $SOURCE_DIR/common/version.h | awk -F\" '{print $2}')

# sed -i -e "s#\[latest\]#$VERSION#g" CHANGELOGS.md

# Add built files to git
git add -f .
git commit --amend -m "T.O.P v$VERSION

version: T.O.P v$TOP_VERSION release
date: $DATETIME
top-dev(priv) master commit: $GIT_HASH
"

# Run tests
#TEST_FILES="tools/"
#cd $SOURCE_DIR
#cp -pR -n --parents $TEST_FILES $BUILD_DIR/
#cd $BUILD_DIR
#RELEASE=1 selfdrive/test/test_onroad.py
#system/manager/test/test_manager.py
#selfdrive/car/tests/test_car_interfaces.py
#rm -rf $TEST_FILES

if [ ! -z "$RELEASE_BRANCH" ]; then
  echo "[-] pushing release T=$SECONDS"
  git push -f origin $RELEASE_BRANCH:$RELEASE_BRANCH
fi

echo "[-] done T=$SECONDS"
