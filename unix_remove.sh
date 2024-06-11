#!/bin/sh
cd build/
sudo xargs rm < install_manifest.txt
cd ..
sudo rm -r build