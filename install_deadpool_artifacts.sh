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
# Back up firmwares
mv ${install_path}/4.9/lib/firmware/video/* ${tmp_dir}/

# Clean target
rm -rf ${install_path}/4.9/*

# Download go/ab artifacts
/google/data/ro/projects/android/fetch_artifact --bid ${FLAGS_build_id} \
  --target kernel_deadpool --nouse_oauth2 'Image'
gzip Image
mv 'Image.gz' ${tmp_dir}/
/google/data/ro/projects/android/fetch_artifact --bid ${FLAGS_build_id} \
  --target kernel_deadpool --nouse_oauth2 '*.ko'
mv '*.ko' ${tmp_dir}/

mkdir -p ${install_path}/4.9/lib/firmware/video/
mkdir -p ${install_path}/4.9/lib/modules/

# Restore firmwares
mv ${tmp_dir}/*.bin ${install_path}/4.9/lib/firmware/video/

# Install go/ab artifacts
mv ${tmp_dir}/Image.gz ${install_path}/4.9/
mv ${tmp_dir}/optee* ${install_path}/4.9/lib/
mv ${tmp_dir}/*.ko ${install_path}/4.9/lib/modules/
