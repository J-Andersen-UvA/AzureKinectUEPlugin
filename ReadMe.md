# Azure Kinect UE Plugin
I've made a simple plugin to use the Azure Kinect SDK within Unreal Engine. It is compromised of two parts:
1. **AzureKinectSimple**
Exposes optical camera and depth camera. Able to return colored texture and depth texture. Build on the Azure Kinect SDK.
2. **AzureKinectBodyTrackingSimple**
Exposes tracking data such as skeleton of a body, closest person, number of people in view. Build on the Azure Kinect Body Tracking SDK.
---

## Requirements
1. The Azure Kinect SDK and Azure Kinect Body Tracking SDK.
We used `v1.4.2` for Azure Kinect SDK and `v1.1.0` for the Azure Kinect Body Tracking SDK.
2. Environment variables:
`AZUREKINECT_SDK` pointing to the root folder of the SDK
`AZUREKINECT_BODY_SDK` pointing to the root folder of the SDK
3. Path variables pointing to the \release\bin folders, and an extra path variable for the `Azure Kinect Body Tracking SDK` at \tools.
4. Enable the MediaFeaturePack for Windows:
```bash
dism /online /add-capability /capabilityname:Media.MediaFeaturePack~~~~0.0.1.0
```

---

## Blueprints
### Azure Kinect Simple
Currently the created C++ Plugin contains ways to read out the `colorTexture`, `depthTexture` and the `depthBuffer`. To use these in a project, you either read them out in C++ or you use the blueprint nodes created in those scripts (`GetColorTexture`, `GetDepthTexture` & `GetDepthData`).
These nodes are childed to the `AzureKinect Component`, an actor needs this component to access this data. Or it needs to get it from another actor.

### Azure Kinect Body Tracking Simple
The following nodes are childed to the `AzureKinectBodyTracking Component`, an actor needs this component to access this data. Or it needs to get it from another actor.

Created nodes:
|Node |Functionality |
|---|---|
| getBodySkeleton | Get an array of joint data |
| getBoneData | Get joint data |
| getTrackedBodyCount | Get amount of people in camera view |

---

## Known Issues
- Its slow when I open the component in the details tab during a run. I think it doesn't like to display the images live in the details tab. Solve for now is to just not open the details tab for the component during a run.
- The depth camera returns an octagon, thats just how it is.
