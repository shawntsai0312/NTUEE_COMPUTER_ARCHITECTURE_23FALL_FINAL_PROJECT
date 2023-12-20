# Computer Architecture 23Fall Final Project
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

#### github command
* Remember to save and git add, git commit before downloading

1. upload
```shell
git add .
git commit -m "your comments"
git push
```

2. download
```shell
git fetch origin
git merge
```

3. new branch
```shell
git checkout -b <branchName>
```

4. switch branch
```shell
git checkout <branchName>
```

5. delete a local branch
 ```shell
 git branch -D <branchName>
 ```

6. delete a remote branch
 ```shell
 git push origin --delete <remoteBranchName>
 ```

7. merge branch
```shell
git checkout main
git merge <branchName>
```