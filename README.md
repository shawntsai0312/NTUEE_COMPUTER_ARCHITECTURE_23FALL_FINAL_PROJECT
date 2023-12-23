# Computer Architecture 23Fall Final Project

##### author : B10901176 蔡弘祥, B10901163 張顥譽

#### Before running

```shell
cd 01_RTL/
source 00_license.sh
```
#### How To Run

```shell
source 01_run.sh I[k]
```

|k      |instruction|
|-------|-----------|
|0      |leaf       |
|1      |fact       |
|2      |hw1        |
|3      |sort       |

##### Example

```shell
source 01_run.sh I0
# CPU will execute "leaf"
```



#### How To Clean files

```shell
source ./01_RTL/99_clean_up.sh
```

#### Some Github Commands

* Remember to save and git add, git commit before downloading

1. upload

```shell
git add .
git commit -m "your comments"
git push
```

2. download

```shell
git fetch origin <branchName>
git checkout main
git merge origin/<branchName>
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
