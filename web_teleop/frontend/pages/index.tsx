import type { NextPage } from 'next'
import Head from 'next/head'
import Image from 'next/image'
import React from 'react'
import ROSLIB from 'roslib'
import Height from '../components/Height'
import styles from '../styles/Home.module.css'

const Home: NextPage = () => {
  const ros = new ROSLIB.Ros({
    url: "ws://localhost:9090"
  })
  ros.on('connection', () => {
      console.log("Connected to websocket server")
  })
  ros.on('error', (error) => {
      console.log("Error connecting to websocket", error)
  })
  ros.on('close', () => {
      console.log("Closed connection to websocket server")
  })
  return (
    <div className={styles.container}>
      <Head>
        <title>Create Next App</title>
        <meta name="description" content="Generated by create next app" />
        <link rel="icon" href="/favicon.ico" />
      </Head>

      <main className={styles.main}>
        <h1 className={styles.title}>
          Welcome to ROS
        </h1>
        <p>Height: <Height ros={ros}/></p> 
      </main>

      <footer className={styles.footer}>
        <a
          href="https://vercel.com?utm_source=create-next-app&utm_medium=default-template&utm_campaign=create-next-app"
          target="_blank"
          rel="noopener noreferrer"
        >
          Powered by{' '}
          <span className={styles.logo}>
            <Image src="/vercel.svg" alt="Vercel Logo" width={72} height={16} />
          </span>
        </a>
      </footer>
    </div>
  )
}

export default Home
