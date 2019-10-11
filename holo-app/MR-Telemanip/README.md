There is an awkward issue with one MRTK asset. Simply, if the final directory name of the "MixedRealityToolkit.Examples.Demos.Utilities.InspectorFields.Inspectors" file is too long and too nested, it can not be found by the Unity Editor which causes built-in compiler errors. The workaround is to clone (or move),
the repo (mr-immersive-telemanipulation folder) closer to the root directory.

Also, Windows path names in general has a limited length. Compiler will raise errors if the directory names are too long. So, after cloning the repo, adjust the main folder name, "mr-immersive-telemanipulation" and put it close to the root directory.

Example Before (not working version):
```
C:\Users\duzce\Desktop\ETH\MR Lab\mr-immersive-telemanipulation\hololens-app\MR Immersive Telemanipulation\\Assets\MixedRealityToolkit.Examples\Demos\Utilities\InspectorFields\Inspectors
```

Example Now (working version):
```
C:\MR\mr-repo\holo-app\MR-Telemanip\Assets\MixedRealityToolkit.Examples\Demos\Utilities\InspectorFields\Inspectors
```

Here's is the full, more official description plus solution
of the issue: https://github.com/microsoft/MixedReality-SpectatorView/tree/master/samples#issue-directorynotfoundexception-during-build
