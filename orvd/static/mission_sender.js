document.getElementById('mission-sender-submit').onclick = submit;

let key = null;

async function submit() {
    let id = document.getElementById('id').value;
    let mission = document.getElementById("myfile").files[0]
    if (id != null && id != '' && mission != null) {
        if (key == null) {
            let key_resp = await fetch('mission_sender/key?id='+id);
            key = await key_resp.text();
            console.log(key);
        }
        const mission_resp = await fetch('mission_sender/fmission_ms?id='+id + "&sig=0xaa", {method:'POST',body:mission});
        const mission_answer = await mission_resp.text();
        alert(mission_answer.split('#')[0]);
    } else {
        alert('Одно из полей пусто.');
    }
}