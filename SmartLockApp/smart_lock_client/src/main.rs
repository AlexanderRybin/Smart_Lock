use std::{sync::Arc, time::Duration};

use proto::{JsonBellState, JsonLockCommand};
use tokio::{io::AsyncBufReadExt, sync::Mutex, time::sleep};
use tokio_util::sync::CancellationToken;

const HTTP_ADDR_BELL: &str = "http://127.0.0.1:3000/bell";
const HTTP_ADDR_LOCK: &str = "http://127.0.0.1:3000/lock";

#[tokio::main]
async fn main() {
    let client = reqwest::Client::new();

    let bell_state = BellState(Mutex::new(false));
    let lock_command = Arc::new(LockCommand(Mutex::new(false)));
    let bell_command = Arc::new(BellCommand(Mutex::new(false)));

    let lock_command_clone = lock_command.clone();
    let bell_command_clone = bell_command.clone();

    let token = CancellationToken::new();
    let token_clone = token.clone();

    tokio::spawn(async move {
        loop {
            let bell_json = JsonBellState { bell: false };

            let bell_request = client.get(HTTP_ADDR_BELL).json(&bell_json).send().await;

            if let Ok(bell_response) = bell_request {
                if bell_response.status().is_success() {
                    match bell_response.json::<JsonBellState>().await {
                        Ok(bell) => {
                            if bell.bell {
                                println!("THE BELL IS RINGING!!!!");
                                bell_state.set().await;
                            } else if bell_state.get().await {
                                bell_state.reset().await;
                            }
                        }
                        Err(_) => {
                            println!("bell response parse error");
                            token_clone.cancel();
                            break;
                        }
                    }
                } else {
                    println!("bell response error");
                    token_clone.cancel();
                    break;
                }
            };

            if bell_command.get().await {
                bell_command.reset().await;
                let bell_json = JsonBellState { bell: false };

                let _ = client.post(HTTP_ADDR_BELL).json(&bell_json).send().await;
            }

            if lock_command.get().await {
                lock_command.reset().await;
                let lock_json = JsonLockCommand { lock: false };

                let _ = client.post(HTTP_ADDR_LOCK).json(&lock_json).send().await;
            }

            sleep(Duration::from_millis(1000)).await;
        }
    });

    tokio::spawn(command_line_handler(
        bell_command_clone,
        lock_command_clone,
        token.clone(),
    ));

    loop {
        tokio::select! {
            _ = token.cancelled() => {
                // The token was cancelled, task can shut down
                break;
            }
        }
    }
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
struct BellCommand(Mutex<bool>);

impl BellCommand {
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

async fn command_line_handler(
    bell_command: Arc<BellCommand>,
    lock: Arc<LockCommand>,
    token: CancellationToken,
) {
    println!("Enter 1 to open lock");
    println!("Enter 2 to reset bell");
    println!("Enter other to shotdown app");
    let mut reader = tokio::io::BufReader::new(tokio::io::stdin());
    loop {
        let mut buffer = Vec::new();
        let fut = reader.read_until(b'\n', &mut buffer).await;
        if fut.is_ok() {
            if buffer[0] == "1".as_bytes()[0] {
                lock.set().await;
            } else if buffer[0] == "2".as_bytes()[0] {
                bell_command.set().await;
                println!("reset bell");
            } else {
                token.cancel();
                break;
            }
        };
    }
}
