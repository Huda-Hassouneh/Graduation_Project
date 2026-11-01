# Configure the current bash shell so Gazebo finds this project's models (model://research_quad,
# model://m400_reference_quad). Source it; do not execute it:
#     source simulation/gazebo/env.sh
# Prepends <repo>/simulation/models to GZ_SIM_RESOURCE_PATH (used by `gz sim`) and SDF_PATH
# (used by `gz sdf`). Existing entries are kept; sourcing twice adds nothing. The path is derived
# from this file's location, so it works from any directory and for any checkout location.

_gp_models="$(cd "$(dirname "${BASH_SOURCE[0]}")/../models" && pwd)"
for _gp_var in GZ_SIM_RESOURCE_PATH SDF_PATH; do
    case ":${!_gp_var:-}:" in
        *":${_gp_models}:"*) ;;
        *) export "${_gp_var}=${_gp_models}${!_gp_var:+:${!_gp_var}}" ;;
    esac
done
unset _gp_models _gp_var
