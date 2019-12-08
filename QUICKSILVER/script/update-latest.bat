set TAG_NAME="latest"

git push origin :refs/tags/%TAG_NAME%
git tag -fa -m "%TAG_NAME%" %TAG_NAME%
git push origin master --tags