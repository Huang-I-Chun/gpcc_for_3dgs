# Using G-PCC to Compress 3D Gaussain

This is a project that extends original G-PCC codec (https://github.com/MPEGGroup/mpeg-pcc-tmc13) to make it available to compress 3D Gaussian, there are some main problems solved in this project:

- 3D Gaussian contains many more attributes than traditional point cloud, so we need to extend the number of attributes that can compress in G-PCC
- 3D Gaussian attributes contain floating points and negative numbers. However, G-PCC only accepts positive integers as input attributes

## Building

### Linux (same to original G-PCC)

- mkdir build
- cd build
- cmake ..
- make

## Example Usage

An example usage script is in `usage/compress_decode.ipynb`. This notebook gives the example of generating `encode.cfg` for G-PCC compression and using `subprocess` to run the G-PCC encode and decode command to compress an example 3D Gaussian `usage/bonsai_50p.ply` we provide, which is the first 50 points of the well-known bonsai 3D Gaussian.

## New Option Different from Original G-PCC

We add the following attributes in the G-PCC encoding/decoding phase:

`"f_dc_0", "f_dc_1", "f_dc_2", "f_rest_0", "f_rest_1","f_rest_2","f_rest_3","f_rest_4","f_rest_5","f_rest_6","f_rest_7","f_rest_8","f_rest_9","f_rest_10","f_rest_11","f_rest_12","f_rest_13","f_rest_14","f_rest_15","f_rest_16","f_rest_17","f_rest_18","f_rest_19","f_rest_20","f_rest_21","f_rest_22","f_rest_23","f_rest_24","f_rest_25","f_rest_26","f_rest_27","f_rest_28","f_rest_29","f_rest_30","f_rest_31","f_rest_32","f_rest_33","f_rest_34","f_rest_35","f_rest_36","f_rest_37","f_rest_38","f_rest_39","f_rest_40","f_rest_41","f_rest_42","f_rest_43","f_rest_44","opacity","scale_0", "scale_1", "scale_2", "rot_0", "rot_1", "rot_2", "rot_3"`

Each attribute will have the following 3 arguments that can be added in encode.cfg, we use `{attr}` to represent the attribute name:

- `attrbute: {attr}`: This informs G-PCC that we want to compress `{attr}`
- `{attr}_qp`: Quantization parameter that uses on this `{attr}`, each attribute can be set individually. The range is related to `--bitdepth`, should be within [4, 51 + 6 * (bitdepth - 8)], but I suggest this value should smaller than 75.
- `{attr}_scale`: The scaling factor of `{attr}`, each attribute can be set individually. The default value is 65535, which is the maximum value of the scaling factor, indicating the best precision in compress/decode.

The argument usage example can ref to `usage/compress_decode.ipynb`.

## Tenichical Details

We ref reflecent encoding method to encode all attributes individually

How we scale, and deal with negative number

We ignore nx, but will add it back while decoding
