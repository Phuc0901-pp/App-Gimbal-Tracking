from flask import Flask, render_template, request, jsonify, send_file
from flask_cors import CORS
from datetime import datetime
import threading
import sqlite3
import os
import csv
import io

app = Flask(__name__, static_folder='static', static_url_path='/static')
CORS(app)

# Data directories
DATA_DIR = 'monitoring_data'
os.makedirs(DATA_DIR, exist_ok=True)

# In-memory cache for real-time display
metrics_history = []
errors_history = []
lock = threading.Lock()

# Database connection
def get_db_path(date=None):
    """Get database path for specific date (defaults to today)"""
    if date is None:
        date = datetime.now().strftime('%Y-%m-%d')
    return os.path.join(DATA_DIR, f'{date}.db')

def init_db(db_path):
    """Initialize database schema"""
    conn = sqlite3.connect(db_path)
    c = conn.cursor()
    
    # Metrics table
    c.execute('''
        CREATE TABLE IF NOT EXISTS metrics (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
            fps REAL,
            inference_time INTEGER,
            frame_drop_rate REAL,
            pan_angle REAL,
            tilt_angle REAL,
            roll_angle REAL,
            ble_connected BOOLEAN,
            ble_latency INTEGER,
            error_count INTEGER,
            uptime INTEGER
        )
    ''')
    
    # Errors table
    c.execute('''
        CREATE TABLE IF NOT EXISTS errors (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
            error_code INTEGER,
            message TEXT
        )
    ''')
    
    # Create indexes for faster queries
    c.execute('CREATE INDEX IF NOT EXISTS idx_metrics_timestamp ON metrics(timestamp)')
    c.execute('CREATE INDEX IF NOT EXISTS idx_errors_timestamp ON errors(timestamp)')
    
    conn.commit()
    conn.close()

# Initialize today's database
init_db(get_db_path())

# Auto-save thread
def auto_save_worker():
    """Background thread to save metrics to database every second"""
    import time
    while True:
        time.sleep(1)  # Save every 1 second
        
        with lock:
            if not metrics_history:
                continue
            
            # Batch insert recent metrics
            db_path = get_db_path()
            init_db(db_path)  # Ensure DB exists
            
            conn = sqlite3.connect(db_path)
            c = conn.cursor()
            
            # Insert metrics (last 100 new entries)
            for metric in metrics_history[-100:]:
                c.execute('''
                    INSERT INTO metrics (
                        timestamp, fps, inference_time, frame_drop_rate,
                        pan_angle, tilt_angle, roll_angle, ble_connected, ble_latency,
                        error_count, uptime
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ''', (
                    metric.get('server_time'),
                    metric.get('fps'),
                    metric.get('inferenceTime'),
                    metric.get('frameDropRate'),
                    metric.get('panAngle'),
                    metric.get('tiltAngle'),
                    metric.get('rollAngle'),
                    metric.get('bleConnected'),
                    metric.get('bleLatency'),
                    metric.get('errorCount'),
                    metric.get('uptime')
                ))
            
            conn.commit()
            conn.close()
            
            # Clear saved metrics from memory (keep last 100 for real-time display)
            if len(metrics_history) > 100:
                metrics_history[:] = metrics_history[-100:]

# Start auto-save thread
save_thread = threading.Thread(target=auto_save_worker, daemon=True)
save_thread.start()

@app.route('/push', methods=['POST'])
def push_metrics():
    """Receive metrics from Android app"""
    try:
        data = request.json
        data['server_time'] = datetime.now().isoformat()
        
        with lock:
            metrics_history.append(data)
            if len(metrics_history) > 1000:
                metrics_history.pop(0)
        
        return jsonify({'status': 'ok', 'count': len(metrics_history)})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 400

@app.route('/push_errors', methods=['POST'])
def push_errors():
    """Receive error log from Android app"""
    try:
        data = request.json
        errors = data.get('errors', [])
        
        with lock:
            errors_history.extend(errors)
            if len(errors_history) > 500:
                errors_history[:] = errors_history[-500:]
        
        # Save to database immediately
        db_path = get_db_path()
        conn = sqlite3.connect(db_path)
        c = conn.cursor()
        
        for err in errors:
            c.execute('''
                INSERT INTO errors (timestamp, error_code, message)
                VALUES (?, ?, ?)
            ''', (
                datetime.fromtimestamp(err['timestamp'] / 1000).isoformat(),
                err['code'],
                err['message']
            ))
        
        conn.commit()
        conn.close()
        
        return jsonify({'status': 'ok'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 400

@app.route('/api/metrics')
def get_metrics():
    """API endpoint for real-time dashboard"""
    with lock:
        return jsonify(metrics_history[-100:] if metrics_history else [])

@app.route('/api/latest')
def get_latest():
    """Get single latest metric"""
    with lock:
        return jsonify(metrics_history[-1] if metrics_history else {})

@app.route('/api/errors')
def get_errors():
    """Get recent errors"""
    with lock:
        return jsonify(errors_history[-50:] if errors_history else [])

@app.route('/api/history')
def get_history():
    """Query historical data from database"""
    date = request.args.get('date', datetime.now().strftime('%Y-%m-%d'))
    start_time = request.args.get('start')
    end_time = request.args.get('end')
    limit = int(request.args.get('limit', 1000))
    
    try:
        db_path = get_db_path(date)
        if not os.path.exists(db_path):
            return jsonify({'error': 'No data for this date'}), 404
        
        conn = sqlite3.connect(db_path)
        conn.row_factory = sqlite3.Row
        c = conn.cursor()
        
        query = 'SELECT * FROM metrics WHERE 1=1'
        params = []
        
        if start_time:
            query += ' AND timestamp >= ?'
            params.append(start_time)
        
        if end_time:
            query += ' AND timestamp <= ?'
            params.append(end_time)
        
        query += ' ORDER BY timestamp DESC LIMIT ?'
        params.append(limit)
        
        c.execute(query, params)
        rows = c.fetchall()
        
        result = [dict(row) for row in rows]
        conn.close()
        
        return jsonify(result)
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/sessions')
def get_sessions():
    """List available database sessions (dates)"""
    sessions = []
    for filename in os.listdir(DATA_DIR):
        if filename.endswith('.db'):
            date = filename.replace('.db', '')
            db_path = os.path.join(DATA_DIR, filename)
            size = os.path.getsize(db_path)
            
            # Count records
            conn = sqlite3.connect(db_path)
            c = conn.cursor()
            c.execute('SELECT COUNT(*) FROM metrics')
            count = c.fetchone()[0]
            conn.close()
            
            sessions.append({
                'date': date,
                'size': size,
                'records': count
            })
    
    sessions.sort(key=lambda x: x['date'], reverse=True)
    return jsonify(sessions)

@app.route('/export/csv')
def export_csv():
    """Export data to CSV file"""
    date = request.args.get('date', datetime.now().strftime('%Y-%m-%d'))
    
    try:
        db_path = get_db_path(date)
        if not os.path.exists(db_path):
            return jsonify({'error': 'No data for this date'}), 404
        
        conn = sqlite3.connect(db_path)
        c = conn.cursor()
        c.execute('SELECT * FROM metrics ORDER BY timestamp')
        rows = c.fetchall()
        columns = [desc[0] for desc in c.description]
        conn.close()
        
        # Create CSV in memory
        output = io.StringIO()
        writer = csv.writer(output)
        writer.writerow(columns)
        writer.writerows(rows)
        
        # Convert to bytes
        mem = io.BytesIO()
        mem.write(output.getvalue().encode('utf-8'))
        mem.seek(0)
        
        return send_file(
            mem,
            mimetype='text/csv',
            as_attachment=True,
            download_name=f'gimbal_metrics_{date}.csv'
        )
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/')
def dashboard():
    """Main dashboard page"""
    return render_template('dashboard.html')

if __name__ == '__main__':
    print("=" * 60)
    print(" GIMBAL TRACKING MONITOR SERVER v2.0")
    print("=" * 60)
    print("Server starting on http://0.0.0.0:5000")
    print(f"\nDatabase: {get_db_path()}")
    print(f"Data directory: {os.path.abspath(DATA_DIR)}")
    print("\nAccess dashboard:")
    print("  - Local:  http://localhost:5000")
    print("  - Network: http://YOUR_LAPTOP_IP:5000")
    print("\nNew Features:")
    print("   SQLite database persistence")
    print("   Auto-save every 1 second")
    print("   Historical data queries")
    print("   CSV export")
    print("   Session management")
    print("\nAndroid App Setup:")
    print("  1. Click 'MONITOR' button")
    print("  2. Enter laptop IP address")
    print("  3. Enable monitoring toggle")
    print("  4. Save")
    print("=" * 60)
    print()
    
    app.run(host='0.0.0.0', port=5000, debug=True, threaded=True)
