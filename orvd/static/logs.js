document.getElementById('submit').onclick = submit;

async function submit() {
    let id = document.getElementById('id').value;
    if (id != null) {
        let logs_resp = await fetch('logs/get_logs?id='+id);
        let logs = await logs_resp.text();
        logs = logs.split('\n');
        let logs_container = document.getElementById('logs-container');
        logs_container.innerHTML = "";
        for (let idx = 0; idx < logs.length; idx++) {
            let logs_text = document.createElement('p');
            logs_text.style.color = 'black';
            logs_text.textContent = logs[idx];
            logs_container.appendChild(logs_text);
        }
    }
}