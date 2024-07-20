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

git commit -m "커밋 메시지"

git pull origin main

git push -u origin main 
