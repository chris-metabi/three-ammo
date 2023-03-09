# three-ammo

A [three.js](https://github.com/mrdoob/three.js/) wrapper for [Ammo.js](https://github.com/kripken/ammo.js/) that can run in a web-worker using Shared Array Buffers or PostMessage. Primarily for use with [Mozilla Hubs](https://github.com/mozilla/hubs).

[side by side comparison of running with/without a worker](https://twitter.com/i/status/1218263836303581184)

Also see:

[three-to-ammo](https://github.com/InfiniteLee/three-to-ammo)

[ammo-debug-drawer](https://github.com/InfiniteLee/ammo-debug-drawer)

-------------------------------

chris-metabi:  This version of three-ammo is being modified to support more ammo js functionality in Mozilla Hubs, such as applyForce among many other desired functions.

Apologies for the git log, I have no way to test this code other than pushing changes to github and then updating my hubs client build... so there will be many trivial experimental changes and less than helpful log messages here.
