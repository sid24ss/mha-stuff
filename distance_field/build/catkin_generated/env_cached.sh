#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Usage: env.sh COMMANDS"
  /bin/echo "Calling env.sh without arguments is not supported anymore. Instead spawn a subshell and source a setup file manually."
  exit 1
else
  . "/usr0/home/venkatrn/groovy_workspace/sandbox/distance_field/build/catkin_generated/setup_cached.sh"
  exec "$@"
fi
