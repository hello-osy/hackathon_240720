해야 할 일:
-roslaunch 파일 실행하면 알아서 주행하도록 해야 함. CMakeFile? 뭐 그런 것도 해야 하나? 이 부분은 잘 모르겠음.

=============

git init

git remote add origin https://github.com/hello-osy/hackathon_240720.git

git branch -M main

git add .

git commit -m "Initial commit"

git push -u origin main 

이렇게 하면 전체 개발 환경 업로드 됨.

============

수정할 것이 있다면?

git add .

git commit -m "0720 commit"

git pull origin main

git push -u origin main 
