
라이다와 rviz 동시 실행
roslaunch ydlidar lidar_view.launch
라이다 실행
roslaunch ydlidar lidar.launch
라이다 값 받아보기
rosrun ydlidar ydlidar_client

git 명령어 정리
2강
git init : 현재 디렉터리를 git으로 버젼관리 가능한 디렉터리로 만든다.
git config user.name "username" : git 사용자 이름 설정
git config user.email "username@naver.com" : git 사용자 이메일 설정
git add . : 스테이징에어리어에 파일을 올린다.
git commit -m "commit_message" : 스테이징에어리어의 파일을 레퍼지토리에 저장한다.
git log : 커밋 로그를 본다.
git diff {commit_id_1} {commit_id_2} : 두 커밋의 차이점을 보여준다.
3강
git reset --{option} {commit_id} : HEAD가 가리키는 커밋을 바꾼다. option -> hard, mixed, soft
git reflog : HEAD가 가리켰던 commit들을 보여준다.
4강
git remote add origin https://lab.hanium.or.kr/21_HF307/21_hf307.git : 원격 저장소를 origin이라는 이름으로 지정한다.
git push -u origin master : 원격 저장소로 업로드한다.
git clone https://lab.hanium.or.kr/21_HF307/21_hf307.git : git에 있는 디렉터리를 가져온다.
git pull : 원격 저장소의 최신 커밋을 가져온다.
