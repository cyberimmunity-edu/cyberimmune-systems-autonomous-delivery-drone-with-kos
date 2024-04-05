//document.getElementById('submit').onclick = submit;
document.getElementById('submit').onclick = submit;

async function submit() {
    let id = document.getElementById('id').value;
    let mission = document.getElementById("myfile").files[0]
    if (id != null && id != '' && mission != null) {
        let key_resp = await fetch('mission_sender/key?id='+id);
        console.log(await key_resp.text());
        let mission_resp = await fetch('mission_sender/fmission_ms?id='+id + "&sig=0xaa", {method:'POST',body:mission});
        console.log(await mission_resp.text());
        alert('Миссия отправлена');
    } else {
        alert('Одно из полей пусто.');
    }
}
