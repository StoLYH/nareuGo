<script setup>
import fourth from "./views/fourth.vue";
import ItemDetailView from "./views/ItemDetailView.vue";
import ItemRegistView from "./views/ItemRegistView.vue";
import NotificationListView from "./views/NotificationListView.vue";
import Login from "./views/Login.vue";
</script>

<template>
  <div class="layout">
    <div class="content">
      <!-- 테스트 2 -->
      <Login />
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
  background-color: #fff8ed;
  font-family: "Pretendard-Regular";
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
} /* 폰트 & CSS 변수 */
@font-face {
  font-family: "Pretendard-Regular";
  src: url("https://cdn.jsdelivr.net/gh/Project-Noonnu/noonfonts_2107@1.1/Pretendard-Regular.woff")
    format("woff");
  font-weight: 400;
  font-style: normal;
}
:root {
  --main: #961f1f;
  --disabled: #dcb6b6;
  --gray: #dbdbdb;
  --warning: #fc6d6d;
  --deepgray: #767676;
}
* {
  box-sizing: border-box;
  font-family: "Pretendard-Regular";
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
  height: 100vh; /* [추가] content가 화면 전체 높이를 차지하도록 설정 */
}
</style>
