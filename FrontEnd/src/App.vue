<script setup>
</script>

<template>
  <div class="layout">
    <div class="content">
      <router-view />
    </div>
  </div>
</template>

<style>
/* reset */
html,
body,
div,
span,
applet,
object,
iframe,
h1,
h2,
h3,
h4,
h5,
h6,
p,
blockquote,
pre,
a,
abbr,
acronym,
address,
big,
cite,
code,
del,
dfn,
em,
img,
ins,
kbd,
q,
s,
samp,
small,
strike,
strong,
sub,
sup,
tt,
var,
b,
u,
i,
center,
dl,
dt,
dd,
ol,
ul,
li,
fieldset,
form,
label,
legend,
table,
caption,
tbody,
tfoot,
thead,
tr,
th,
td,
article,
aside,
canvas,
details,
embed,
figure,
figcaption,
footer,
header,
hgroup,
menu,
nav,
output,
ruby,
section,
summary,
time,
mark,
audio,
video {
  margin: 0;
  padding: 0;
  border: 0;
  font-size: 100%;
  font: inherit;
  vertical-align: baseline;
}
body {
  line-height: 1;
  /* 전체 페이지 배경: 헤더/로그인 버튼 그라디언트와 조화되는 라이트 블루 톤 */
  background: linear-gradient(180deg, #EAF3FB 0%, #F5F9FC 100%);
  font-family: "Pretendard-Regular", "Noto Sans KR", -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
}
ol,
ul {
  list-style: none;
}
blockquote,
q {
  quotes: none;
}
blockquote::before,
blockquote::after,
q::before,
q::after {
  content: "";
}
table {
  border-collapse: collapse;
  border-spacing: 0;
}
/* === Global caret and selection rules to avoid stray carets on non-inputs === */
/* 기본적으로 전체 페이지에서 캐럿을 숨기고, 실제 입력 가능한 요소에서만 표시합니다. */
/* 모든 요소에 대해 기본 캐럿 숨김 처리 */
*:not(input):not(textarea):not([contenteditable]):not([contenteditable="true"]) {
  caret-color: transparent;
}
/* 입력 가능한 요소에 대해서만 캐럿 표시 */
input, textarea, [contenteditable]:not([contenteditable="false"]) {
  caret-color: auto; /* 입력창에서는 정상 표시 */
}
/* 비인터랙티브 요소에 포커스가 가는 경우의 외곽선 제거 (접근성 영향 없음: 버튼/링크에는 적용하지 않음) */
:where(p, span, img, svg, strong, em, h1, h2, h3, h4, h5, h6):focus {
  outline: none;
}
/* UI 컨트롤에서 텍스트 드래그 방지 (불필요한 선택 방지) */
button, .nav-item, .slider-nav, .like-button, .chat-button {
  user-select: none;
}
/* 페이지 전반에서 비입력 요소의 텍스트 선택 및 캐럿 표시를 억제 */
html, body, .layout, .content {
  -webkit-user-select: none;
  -moz-user-select: none;
  -ms-user-select: none;
  user-select: none;
}
/* 입력/에디터 영역에서는 정상적으로 텍스트 선택 및 캐럿 표시 */
input, textarea, [contenteditable="true"], [contenteditable]:not([contenteditable="false"]) {
  -webkit-user-select: text;
  -moz-user-select: text;
  -ms-user-select: text;
  user-select: text;
}
/* 폰트 & CSS 변수 */
@import url('https://fonts.googleapis.com/css2?family=Noto+Sans+KR:wght@400;500;600;700&display=swap');

@font-face {
  font-family: "Pretendard-Regular";
  src: url("https://cdn.jsdelivr.net/gh/Project-Noonnu/noonfonts_2107@1.1/Pretendard-Regular.woff")
    format("woff");
  font-weight: 400;
  font-style: normal;
}
:root {
  --main: #4682b4;
  --disabled: #dcb6b6;
  --gray: #dbdbdb;
  --warning: #fc6d6d;
  --deepgray: #767676;
}
* {
  box-sizing: border-box;
  font-family: "Pretendard-Regular", "Noto Sans KR", -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
}
a {
  text-decoration: none;
  color: inherit;
}
button {
  background: none;
  border: none;
  padding: 0;
  margin: 0;
  cursor: pointer;
}
.a11y-hidden {
  clip: rect(1px, 1px, 1px, 1px);
  clip-path: inset(50%);
  width: 1px;
  height: 1px;
  margin: -1px;
  overflow: hidden;
  padding: 0;
  position: absolute;
}

.layout {
  width: 100%;
  max-width: 390px;
  margin: 0 auto;
  min-height: 100vh;
  display: flex;
  justify-content: center;
  align-items: flex-start;
  background-color: #f8f9fa;
  box-shadow: rgba(100, 100, 111, 0.2) 0px 7px 29px 0px;
}

.content {
  width: 100%;
  /* height: 100vh;  */
}
</style>
