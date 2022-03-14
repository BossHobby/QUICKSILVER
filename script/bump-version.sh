#! /bin/bash

PREFIX="v"

DEVELOP=false

if [ "$(git branch --show-current)" = "develop" ]; then
  DEVELOP=true
fi

find_latest_semver() {
  pattern="^$PREFIX([0-9]+\.[0-9]+\.[0-9]+)(-dev)?$"
  versions=$(for tag in $(git tag); do
    [[ "$tag" =~ $pattern ]] && echo "${BASH_REMATCH[1]}"
  done)
  if [ -z "$versions" ];then
    echo 0.0.0
  else
    echo "$versions" | tr '.' ' ' | sort -nr -k 1 -k 2 -k 3 | tr ' ' '.' | head -1
  fi
}

bump() {
  latest_semver=$(find_latest_semver)
  latest_ver=${PREFIX}${latest_semver}
  latest_commit=$(git rev-parse "${latest_ver}" 2>/dev/null )
  head_commit=$(git rev-parse HEAD)

  if [ "$latest_commit" = "$head_commit" ]; then
    echo "refusing to tag; $latest_ver already tagged for HEAD ($head_commit)"
  else
    next_major=$(($(echo $latest_semver | cut -d "." -f 1) + "$1"))
    next_minor=$(($(echo $latest_semver | cut -d "." -f 2) + "$2"))
    next_patch=$(($(echo $latest_semver | cut -d "." -f 3) + "$3"))

    if [ $1 -gt 0 ]; then
      next_minor="0"
      next_patch="0"
    fi

    if [ $2 -gt 0 ]; then
      next_patch="0"
    fi

    next_ver="${PREFIX}${next_major}.${next_minor}.${next_patch}"

    if [ $DEVELOP = true ]; then
      next_ver="$next_ver-dev"
    fi

    echo "tagging $next_ver $head_commit"
    git tag "$next_ver" $head_commit
    git push --tags
  fi
}

usage() {
  echo "Usage: bump-version {major|minor|patch}"
  echo "Bumps the semantic version field by one for a git-project."
  exit 1
}

case "$1" in
  major) bump 1 0 0;;
  minor) bump 0 1 0;;
  patch) bump 0 0 1;;
  *) usage
esac