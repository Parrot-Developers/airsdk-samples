#!/bin/bash

function usage()
{
    echo "$0 <in_archive> <out_archive> <sign_workspace_dir>"
    echo ""
    echo "Re-sign an archive with production keys"
    echo "<in_archive> : input archive"
    echo "<out_archive>: output archive"
    echo "<sign_workspace_dir>: workspace where to get all the required tools to sign a release"
}

if [ "$1" = "-h" -o "$1" = "--help" ]; then
    usage
    exit 0
fi

if [ "$#" != 3 ]; then
    usage
    exit 1
fi

set -e

readonly IN_ARCHIVE=$1
readonly OUT_ARCHIVE=$2
readonly SIGN_WORKSPACE_DIR=$3

# Search the sign tools in the provided signature workspace
readonly SIGN_TOOLS=$(find ${SIGN_WORKSPACE_DIR} -type d -name dragon_buildext_sign -print -quit)

if [ "${SIGN_TOOLS}" = "" ]; then
    echo >&2 "Could not find the sign tools directory."
    exit 1
fi

[ -n "${YHSM_SESSION_ID}" ] || (echo "Missing YHSM_SESSION_ID in env"; exit 1)
[ -n "${YHSM_CREDENTIAL_FILE}" ] || (echo "Missing YHSM_CREDENTIAL_FILE in env"; exit 1)

# Key names
readonly DEV_KEY="dev_key"
readonly PROD_KEY="prod_key"

# Setup temp dir that will be deleted at the end
readonly TMP_DIR=$(mktemp -d --suffix .sign)
trap "rm -rf ${TMP_DIR}" EXIT SIGINT SIGTERM

# Some variables
readonly TMP_ARCHIVE=${TMP_DIR}/$(basename ${IN_ARCHIVE})
readonly TMP_ARCHIVE_TAR=${TMP_ARCHIVE%.gz}
readonly REF_SIGNATURE_FILE="signature.ecdsa-dev"
readonly TAR_CMD="tar -C ${TMP_DIR} -f ${TMP_ARCHIVE_TAR}"

# Prepare the archive in temp dir
cp ${IN_ARCHIVE} ${TMP_ARCHIVE}
gunzip ${TMP_ARCHIVE}

# Get list of filenames to sign using reference signature file
${TAR_CMD} --extract ${REF_SIGNATURE_FILE}
filenames=$(grep filenames ${TMP_DIR}/${REF_SIGNATURE_FILE} | cut -d= -f2)
[ -n "${filenames}" ] || (echo "No reference signature with 'filenames'"; exit 1)

# Sign an archive as given filename and key
function sign()
{
    local SIGNATURE_FILENAME=$1
    local KEY=$2

    ${SIGN_TOOLS}/sign.py \
        --filenames ${filenames} \
        -n ${SIGNATURE_FILENAME} \
        -k ${KEY} \
        --hash sha512 \
        ${TMP_ARCHIVE_TAR}
}

# Remove previous signature
${TAR_CMD} --delete --wildcards signature*

echo "Re-sign the archive in ${OUT_ARCHIVE}"
sign signature.ecdsa-dev ${DEV_KEY}
sign signature.ecdsa ${PROD_KEY}

# Finish the archive
gzip ${TMP_ARCHIVE_TAR}
cp -f ${TMP_ARCHIVE} ${OUT_ARCHIVE}
