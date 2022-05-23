#!/bin/bash -eu

source gbash.sh || exit

DEFINE_string --required build_id "" "GMS Build Id on go/ab"
DEFINE_string --required install_path "" "The target deadpool-kernel path"

gbash::init_google "${@}"

if [[ "${FLAGS_build_id}" == "" || "${FLAGS_install_path}" == "" ]]; then
  gbash::quiet_die "Please specify build_id and install_path"
fi

# Working dir
tmp_dir=$(mktemp -d -t deadpool-kernel-XXXXXXXXXX)

# TODO: firmware should be built from kernel
# Back up deadpool.dtb and dtbo.img
cp -rf ${FLAGS_install_path}/4.9/deadpool.dtb ${tmp_dir}/
cp -rf ${FLAGS_install_path}/4.9/dtbo.img ${tmp_dir}/

# Clean target
rm -rf ${FLAGS_install_path}/4.9/*

# Download go/ab artifacts
/google/data/ro/projects/android/fetch_artifact --bid ${FLAGS_build_id} \
  --target kernel_deadpool --nouse_oauth2 'Image'
gzip Image
mv Image.gz ${tmp_dir}/

/google/data/ro/projects/android/fetch_artifact --bid ${FLAGS_build_id} \
  --target kernel_deadpool --nouse_oauth2 'unstripped/*.ko'
mv *.ko ${tmp_dir}/

/google/data/ro/projects/android/fetch_artifact --bid ${FLAGS_build_id} \
  --target kernel_deadpool --nouse_oauth2 'unstripped/*.bin'
mv *.bin ${tmp_dir}/

mkdir -p ${FLAGS_install_path}/4.9/lib/firmware/video/
mkdir -p ${FLAGS_install_path}/4.9/lib/modules/

# Restore deadpool.dtb and dtbo.img
mv ${tmp_dir}/deadpool.dtb ${FLAGS_install_path}/4.9/
mv ${tmp_dir}/dtbo.img ${FLAGS_install_path}/4.9/

# Install go/ab artifacts
mv ${tmp_dir}/Image.gz ${FLAGS_install_path}/4.9/
mv ${tmp_dir}/*.bin ${FLAGS_install_path}/4.9/lib/firmware/video/
mv ${tmp_dir}/optee* ${FLAGS_install_path}/4.9/lib/
mv ${tmp_dir}/*.ko ${FLAGS_install_path}/4.9/lib/modules/
