#!/bin/bash

HOME=".."
PROJECTS="test"
TEST="test"

MAKE_COMMAND="scons board=openmote-cc2538 project="
CLEAN_COMMAND="scons -c board=openmote-cc2538 project="

cd $HOME
HOME=$(pwd)

cd $PROJECTS
for PROJECT in *; do
    if [[ -d $PROJECT ]]; then
        
        cd $HOME
        
        echo -ne "Cleaning $PROJECT..."
        OUTPUT=$($CLEAN_COMMAND$PROJECT 2>&1)
        echo -e "ok!"
        
        echo -ne "Building $PROJECT..."
        OUTPUT=$($MAKE_COMMAND$PROJECT 2>&1) 
        
        if [[ $OUTPUT == *rror* ]]; then
            echo -e "$OUTPUT" > $HOME/$TEST/$PROJECT.log
            echo -e "error!"
            ERROR=true
        else
            echo -e "ok!"
        fi
        
        cd $PROJECTS
        
    fi
done

if [[ "$ERROR" == true ]]; then
    echo "Tests finished with errors!"
    exit 1
else
    echo "Tests finished successfully!"
    exit 0
fi

