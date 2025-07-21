find . -type f -print0 | while IFS= read -r -d $'\0' f; do
    echo "--- 파일명: $f ---" # 파일 이름 출력
    cat "$f" # 파일 내용 출력
    echo # 각 파일 내용 사이에 빈 줄 추가 (가독성을 위해)
done
