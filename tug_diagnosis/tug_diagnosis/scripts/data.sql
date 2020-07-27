
create table diagnosis_data (
	id integer primary key autoincrement,
	stamp text not null,
	seq integer not null,
	frame_id text not null,
	msg_type text not null,
	msg_diagnoses text
);

create table observation_data (
	id integer primary key autoincrement,
	header text not null,
        resources text not null,
        observation text not null,
        observation_msg text not null,
        verbose_observation_msg text not null
);
