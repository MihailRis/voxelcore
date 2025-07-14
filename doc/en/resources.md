# Resources

Resources include:
- cameras
- effects slots
- framebuffers
- and other limited resources

At the moment only the following are implemented:
- camera - **camera**.
- post-effect - **effect slot**.

The resources requested by the pack are specified through the *resources.json* file in the format:
```json
{
    "resource-type": [
        "resources",
        "names"
    ]
}
```

After loading the pack, resource names will have the pack prefix. For example camera
*cinematic* in the base package will be *base:cinematic*.
