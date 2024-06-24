# License Registration

* Get the feature id of your working machine.
```base
    cd Motion_Planning/Manipulation/AnyGrasp/license_registration
    ./license_checker -f
```
* Fill in the [form](https://forms.gle/XVV3Eip8njTYJEBo6) to apply for license, which requires the machine feature id.
* You will get a `.zip` file that contains license. The folder structure is as follows (see [sample_license](sample_license) for example):
```base
    license/
       |-- licenseCfg.json
       |-- [your_name].public_key
       |-- [your_name].signature
       |-- [your_name].lic
```
* You can check license states via
```base
    ./license_checker -c license/licenseCfg.json
```
* Name this license folder as `license` and place this inside the `/ok-robot-manipulation/src/` directory.

* Along with the license you will receive the `checkpoint.tar`. Place this in `/ok-robot-manipulation/src/checkpoints/`
