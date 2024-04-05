document.getElementById('submit').onclick = submit;

async function submit() {
    let login = document.getElementById('login').value;
    let password = document.getElementById("password").value;
    if (login != null && login != '' && password != null && password != '') {
        let auth_resp = await fetch('auth?login='+login + '&password='+password);
        let access_token = await auth_resp.text();
        if (auth_resp.status == 200) {
            window.location.href = '/admin?token=' + access_token;
        } else {
            alert('Неправильный логин или пароль.');
        }
    } else {
        alert('Одно из полей пусто.');
    }
}