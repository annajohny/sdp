from flask import Flask, request, render_template,g
import sqlite3

app = Flask(__name__)

# connect db
def connect_db():
    app.app_context().push()
    sql = sqlite3.connect('diagnoses.db')
    sql.row_factory = sqlite3.Row
    return sql

# check the sqlite db
def get_db():
    app.app_context().push()
    if not hasattr(g,'sqlite3_db'):
    	g.sqlite_db = connect_db()
    return g.sqlite_db

# display on the web page
@app.route('/', methods=['GET', 'POST'])
def display():
    db = get_db()
    cur_diagnosis = db.execute('select stamp, seq, frame_id, msg_type, msg_diagnoses from diagnosis_data order by seq DESC')
    cur_observation = db.execute('select header, resources, observation,observation_msg,verbose_observation_msg from observation_data order by id DESC')
    
    results_diagnosis = cur_diagnosis.fetchall()
    results_observation = cur_observation.fetchall()
    
    result = zip(results_diagnosis,results_observation)
    

    return render_template('home.html',context=result)


if __name__ == '__main__':
    app.run(debug=True)
