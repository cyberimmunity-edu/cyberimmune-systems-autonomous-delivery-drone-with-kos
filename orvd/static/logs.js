document.getElementById('submit').onclick = submit;
document.getElementById('submit-speed').onclick = submitSpeed;

let chartInstance = null;

async function submit() {
    let id = document.getElementById('id').value;
    if (id != null) {
        let chartContainer = document.getElementById('speed-chart').getContext('2d');
        chartContainer.clearRect(0, 0, chartContainer.canvas.width, chartContainer.canvas.height);
        if (chartInstance) {
            chartInstance.destroy();
            chartInstance = null;
        }

        let logs_resp = await fetch('logs/get_logs?id=' + id);
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

async function submitSpeed() {
    let id = document.getElementById('id').value;
    if (id != null) {
        let logs_container = document.getElementById('logs-container');
        logs_container.innerHTML = "";

        let response = await fetch('logs/get_telemetry_csv?id=' + id);
        let csv = await response.text();
        let rows = csv.split('\n').slice(1);
        let times = [];
        let speeds = [];
        rows.forEach(row => {
            let cols = row.split(',');
            if (cols.length >= 7) {
                times.push(new Date(cols[0]));
                speeds.push(parseFloat(cols[7]));
            }
        });
        drawChart(times, speeds);
    }
}

function drawChart(labels, data) {
    let ctx = document.getElementById('speed-chart').getContext('2d');
    if (chartInstance) {
        chartInstance.destroy();
    }
    chartInstance = new Chart(ctx, {
        type: 'line',
        data: {
            labels: labels,
            datasets: [{
                label: 'Speed over Time',
                data: data,
                borderColor: 'rgba(75, 192, 192, 1)',
                borderWidth: 1,
                fill: false
            }]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            scales: {
                x: {
                    type: 'time',
                    time: {
                        unit: 'minute'
                    }
                },
                y: {
                    beginAtZero: true
                }
            }
        }
    });
}