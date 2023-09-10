use std::{error::Error, io, net::SocketAddr, sync::Arc, time::Duration};

use proto::{JsonBellState, JsonLockCommand};
use tokio::{
    io::AsyncBufReadExt,
    sync::Mutex,
    time::{self, sleep},
};

use axum::extract::State;
use axum::response::IntoResponse;
use axum::routing::get;
use axum::routing::post;
use axum::Json;
use axum::{http::StatusCode, response::Html, Router};
use tokio_util::sync::CancellationToken;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let ports = serialport::available_ports().expect("No ports found!");

    println!();
    println!("Select port:");
    let mut n: usize = 1;
    for p in &ports {
        println!("{}: {}", n, p.port_name);
        n += 1;
    }

    let port_name = &ports[select_port(n) - 1].port_name;

    let mut port = serialport::new(port_name, 115_200)
        .timeout(Duration::from_millis(10))
        .open()?;

    let mut buf: [u8; 1] = [0];

    let bell = Arc::new(BellState(Mutex::default()));
    let lock = Arc::new(LockCommand(Mutex::default()));

    let bell_clone = bell.clone();
    let lock_clone = lock.clone();

    //
    //                              Serial Server
    //
    let serial_server = async move {
        loop {
            let mut command_serial: [u8; 1] = [SerialCommand::GetMes.into()];

            let _ = port.write(&command_serial);

            buf[0] = 2;

            let response = port.read(&mut buf);

            if response.is_ok() && buf[0] == 1 {
                println!("serial server received bell");

                bell_clone.set().await;

                let mut interval = time::interval(Duration::from_millis(100));

                'inner: for _i in 0..50 {
                    interval.tick().await;

                    if !(bell_clone.get().await) {
                        println!("serial server received reset bell");
                        break 'inner;
                    }

                    if lock_clone.get().await {
                        println!("serial server received open lock");
                        command_serial[0] = SerialCommand::Open.into();
                        let _ = port.write(&command_serial);

                        lock_clone.reset().await;
                        bell_clone.reset().await;
                        break 'inner;
                    }
                }

                if bell_clone.get().await {
                    bell_clone.reset().await;
                    println!("serial server reset bell");
                }

                sleep(Duration::from_millis(1000)).await;
            }

            if lock_clone.get().await {
                println!("serial server received open lock");
                command_serial[0] = SerialCommand::Open.into();
                let _ = port.write(&command_serial);

                lock_clone.reset().await;
            }

            sleep(Duration::from_millis(100)).await;
        }
    };

    let token = CancellationToken::new();

    let bell_clone = bell.clone();
    let lock_clone = lock.clone();

    tokio::spawn(serial_server);
    tokio::spawn(command_line_client(bell_clone, lock_clone, token.clone()));

    let state = (bell.clone(), lock.clone());

    let addr = SocketAddr::from(([127, 0, 0, 1], 3000));
    let app = Router::new()
        .route("/", get(index))
        .route("/lock", post(post_lock))
        .route("/bell", post(post_bell))
        .route("/bell", get(get_bell))
        .with_state(state);

    let http_server = async move {
        axum::Server::bind(&addr)
            .serve(app.into_make_service())
            .await
    };
    tokio::spawn(http_server);

    loop {
        tokio::select! {
            _ = token.cancelled() => {
                // The token was cancelled, task can shut down
                break;
            }
        }
    }

    Ok(())
}

#[derive(Clone, Copy, PartialEq)]
enum SerialCommand {
    Open,
    GetMes,
}

impl Into<u8> for SerialCommand {
    fn into(self) -> u8 {
        match self {
            SerialCommand::Open => 0x31,
            SerialCommand::GetMes => 0x32,
        }
    }
}

type AppState = (Arc<BellState>, Arc<LockCommand>);

//
//                                Http Server
//
async fn index(State(_): State<AppState>) -> Html<&'static str> {
    let r = String::from("<h1> SmartLock index</h1>");

    Html(Box::leak(r.into_boxed_str()))
}

async fn post_lock(
    State(state): State<AppState>,
    Json(payload): Json<JsonLockCommand>,
) -> impl IntoResponse {
    if !(payload.lock) {
        state.1.set().await;
    }

    StatusCode::OK
}

async fn post_bell(
    State(state): State<AppState>,
    Json(payload): Json<JsonBellState>,
) -> impl IntoResponse {
    if !(payload.bell) {
        state.0.reset().await;
    }
    StatusCode::OK
}

async fn get_bell(
    State(state): State<AppState>,
    Json(_): Json<JsonBellState>,
) -> impl IntoResponse {
    let ans_json = JsonBellState {
        bell: state.0.get().await,
    };
    Json(ans_json).into_response()
}

//
//                              App state Struct
//
#[derive(Default)]
struct BellState(Mutex<bool>);

impl BellState {
    async fn reset(&self) {
        let mut n = self.0.lock().await;
        *n = false;
    }

    async fn set(&self) {
        let mut n = self.0.lock().await;
        *n = true;
    }

    async fn get(&self) -> bool {
        let n = self.0.lock().await;
        *n
    }
}

#[derive(Default)]
struct LockCommand(Mutex<bool>);

impl LockCommand {
    async fn reset(&self) {
        let mut n = self.0.lock().await;
        *n = false;
    }

    async fn set(&self) {
        let mut n = self.0.lock().await;
        *n = true;
    }

    async fn get(&self) -> bool {
        let n = self.0.lock().await;
        *n
    }
}

//
//                                Command Line Client
//
async fn command_line_client(
    bell: Arc<BellState>,
    lock: Arc<LockCommand>,
    token: CancellationToken,
) {
    println!("Enter 1 to open lock");
    println!("Enter 2 to reset bell");
    println!("Enter other to shotdown server");
    let mut reader = tokio::io::BufReader::new(tokio::io::stdin());
    loop {
        let mut buffer = Vec::new();
        let fut = reader.read_until(b'\n', &mut buffer).await;
        if fut.is_ok() {
            if buffer[0] == "1".as_bytes()[0] {
                lock.set().await;
            } else if buffer[0] == "2".as_bytes()[0] {
                bell.reset().await;
            } else {
                token.cancel();
                break;
            }
        };
    }
}

//
//                                Select Port Function
//
fn select_port(cnt_port: usize) -> usize {
    loop {
        let mut input = String::new();
        io::stdin().read_line(&mut input).unwrap();
        let num = input.trim().parse::<usize>();

        if let Ok(port_num) = num {
            if port_num < cnt_port && port_num > 0 {
                return port_num;
            } else {
                println!("Select port again");
            }
        } else {
            println!("Select port again");
        }
    }
}
