# I/O Block Layer Code Reading Memo
# Block I/O Controller
Block I/O Controller is Cgroup Subsystem which has benn included into Linux Kernel since v2.6.33

## Build Option
-BLOCK
Enable The Block Layer

-CGROUPS
Control Group Support 

-BLK_CGROUP
Block IO controller
Depends on:CGROUPS && BLOCK

-DEBUG_BLK_CGROUP
Enable Block IO controller debugging
Depends on:CGROUPS && BLK_CGROUP

-CFQ_GROUP_IOSCHED
CFQ group scheduling support
Depends on:BLOCK && IOCHED_CFQ && BLK_CGROUP


