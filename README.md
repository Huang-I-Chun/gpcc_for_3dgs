# Using GPCC to Compress 3D Gaussain

This is a project that extends original GPCC codec (https://github.com/MPEGGroup/mpeg-pcc-tmc13) to make it available to compress 3D Gaussian, there are some main problems solved in this project:

- 3D Gaussian contains many more attributes than traditional point cloud, so we need to extend the number of attributes that can compress in GPCC
- 3D Gaussian attributes contain floating points and negative numbers. However, GPCC only accepts positive integers as input attributes

## Building

### Linux (same to original G-PCC)

- mkdir build
- cd build
- cmake ..
- make

## Example Usage

This TMC13 codec implementation encodes frame sequences. A single binary
contains the encoder and decoder implementation, with selection using
the `--mode` option. Documentation of options is provided via the
`--help` command line option.

### Runtime configuration and configuration files

All command line parameters may be specified in a configuration file.
A set of configuration file templates compliant with the current Common
Test Conditions is provided in the cfg/ directory.

### Example

To generate the configuration files, run the gen-cfg.sh script:

```console
mpeg-pcc-tmc13/cfg$ ../scripts/gen-cfg.sh --all
```

An example script (`scripts/Makefile.tmc13-step`) demonstrates how
to launch the encoder, decoder and metric software for a single
input frame. The VERBOSE=1 make variable shows the detailed command
execution sequence. Further documentation of the parameters are
contained within the script.

The following example encodes and decodes frame 0100 of the sequence
`Ford_01_q_1mm`, making use of the configuration file
`cfg/lossy-geom-no-attrs/ford_01_q1mm/r01/encoder.cfg` and storing
the intermediate results in the output directory
`experiment/lossy-geom-no-attrs/ford_01_q1mm/r01/`.

```console
mpeg-pcc-tmc13$ make -f $PWD/scripts/Makefile.tmc13-step \
    -C experiment/lossy-geom-no-attrs/ford_01_q1mm/r01/ \
    VPATH=$PWD/cfg/octree-predlift/lossy-geom-no-attrs/ford_01_q1mm/r01/ \
    ENCODER=$PWD/build/tmc3/tmc3 \
    DECODER=$PWD/build/tmc3/tmc3 \
    PCERROR=/path/to/pc_error \
    SRCSEQ=/path/to/Ford_01_q_1mm/Ford_01_vox1mm-0100.ply \
    NORMSEQ=/path/to/Ford_01_q_1mm/Ford_01_vox1mm-0100.ply

  [encode]  Ford_01_vox1mm-0100.ply.bin <- /path/to/Ford_01_q_1mm/Ford_01_vox1mm-0100.ply
  [md5sum]  Ford_01_vox1mm-0100.ply.bin.md5
  [md5sum]  Ford_01_vox1mm-0100.ply.bin.ply.md5
  [decode]  Ford_01_vox1mm-0100.ply.bin.decoded.ply <- Ford_01_vox1mm-0100.ply.bin
  [md5sum]  Ford_01_vox1mm-0100.ply.bin.decoded.ply.md5
  [metric]  Ford_01_vox1mm-0100.ply.bin.decoded.ply.pc_error <- Ford_01_vox1mm-0100.ply.bin.decoded.ply
```

## Intra and inter prediction

The yaml files stored directly under the cfg/ folder correspond to intra prediction, and yaml files stored under cfg/inter/ folder correspond to inter prediction. The gen-cfg.sh script is updated such that intra/inter prediction may be specified as an additional option to produce the configuration files corresponding to intra/inter prediction; alternately, the "--all" option may be used to generate the configuration for intra and inter prediction for all tool configurations.

After running the gen-cfg.sh script, the configuration files for intra and inter prediction are generated in separate folders. The configuration files corresponding to inter prediction are generated in folders with "-inter" suffix. For example, configuration files corresponding to octree and predicting/lifting transform using intra prediction are generated in the folder octree-predlift/ (as was the case in some earlier versions of the test model), and configuration files corresponding to octree and predicting/lifting transform using inter prediction are generated in the folder octree-predlift-inter/.
