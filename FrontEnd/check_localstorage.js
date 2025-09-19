console.log('=== localStorage 전체 내용 ==='); for(let i=0; i<localStorage.length; i++) { const key = localStorage.key(i); const value = localStorage.getItem(key); console.log(, value); }
