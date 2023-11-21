# Computer Architecture 23Fall
##### author : B10901176 蔡弘祥, B10901163 張顥譽
#### Before running
```shell
source ./01_RTL/00_license.f
```
##### modified ./HW2/01_RTL/01_run.f
```
vcs ../00_TB/HW2_tb.v HW2.v -full64 -R -debug_access+all +v2k +notimingcheck +define+I<number>
```


|number |instruction|
|-------|-----------|
|0      |add        |
|1      |sub        |
|2      |and        |
|3      |or         |
|4      |slt        |
|5      |sra        |
|6      |mul        |
|7      |div        |

##### Example
```
vcs ../00_TB/HW2_tb.v HW2.v -full64 -R -debug_access+all +v2k +notimingcheck +define+I0
```

ALU will execute addition
  
#### How to Run
```shell
source ./01_RTL/01_run.f
```

#### How to Clean files
```shell
source ./01_RTL/99_clean_up.f
```