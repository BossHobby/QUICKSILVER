#! /bin/bash
TAG_NAME="latest"

git push origin :refs/tags/$TAG_NAME
git tag -fa -m "$TAG_NAME" $TAG_NAME
git push origin master --tags

if which hub; then 
  hub release -f "%T (%S) %n" --include-drafts | grep " (draft)" | awk '{print $1}' | xargs -t -n1 hub release delete
fi