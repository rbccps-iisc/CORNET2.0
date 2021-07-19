#!/bin/bash
image_and_tag="$1"
image_and_tag_array=(${image_and_tag//:/ })
if [[ "$(sudo docker images ${image_and_tag_array[0]} | grep ${image_and_tag_array[1]} 2> /dev/null)" != "" ]]; then
  echo "exists"
else
  echo "doesn't exist"
fi
