git init

git remote add origin https://github.com/hello-osy/hackathon_240720.git

git branch -M main

git add .

git commit -m "Initial commit"

git push -u origin main 

이렇게 하면 전체 개발 환경 업로드 됨.

============

수정할 것이 있다면?

git pull origin main

git add .

git push -u origin main 


만약 병합 충돌이 발생하면 충돌을 해결하고 아래 명령어를 실행합니다.

git pull origin main

git commit -m "Resolve merge conflict"

git push -u origin main
