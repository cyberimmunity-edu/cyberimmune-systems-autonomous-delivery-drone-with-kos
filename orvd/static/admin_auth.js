document.getElementById('auth-submit-button').onclick = submit;

async function submit(event) {
    event.preventDefault();
    let login = document.getElementById('login').value;
    let password = document.getElementById("password").value;
    let nextUrl = document.getElementById("next-url").value;
    if (login != null && login != '' && password != null && password != '') {
        let auth_resp = await fetch('auth?login=' + login + '&password=' + password);
        let access_token = await auth_resp.text();
        if (auth_resp.status == 200) {
            const redirectUrl = nextUrl || '/admin';
            window.location.href = redirectUrl + '?token=' + access_token;
        } else {
            alert('Неправильный логин или пароль.');
        }
    } else {
        alert('Одно из полей пусто.');
    }
}