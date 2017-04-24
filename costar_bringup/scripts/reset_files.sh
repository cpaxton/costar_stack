#!/usr/bin/env bash

mv $HOME/.costar user_$1_files
git clone git@github.com:cpaxton/costar_files.git $HOME/.costar --branch study2

