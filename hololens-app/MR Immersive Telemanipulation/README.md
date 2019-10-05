There is an awkward issue with one MRTK asset. Simply, if the final directory name of the "MixedRealityToolkit.Examples.Demos.Utilities.InspectorFields.Inspectors"
file is too long and too nested, it can not be found by the Unity Editor which causes built-in compiler errors. The workaround is to clone (or move),
the repo (mr-immersive-telemanipulation folder) to closer to root directory. Here's is the full, more official description plus solution
of the issue: https://github.com/microsoft/MixedReality-SpectatorView/tree/master/samples#issue-directorynotfoundexception-during-build
