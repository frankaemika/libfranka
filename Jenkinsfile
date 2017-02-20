#!groovy

node {
  step([$class: 'StashNotifier'])

  try {
    stage('Checkout') {
      checkout scm
    }

    stage('Build in debug mode') {
      sh 'scripts/ci/debug-build.sh'
    }

    stage('Build with code coverage') {
      sh 'scripts/ci/coverage-build.sh'
    }

    stage('Build in release mode') {
      sh 'scripts/ci/release-build.sh'
    }

    stage('Build examples') {
      sh 'scripts/ci/examples-build.sh'
    }

    stage('Archive results') {
      junit 'build/test_results/*.xml'
      step([$class: 'Mailer', notifyEveryUnstableBuild: true, sendToIndividuals: true])

      archive 'build-release/*.deb, build-release/*.tar.gz'
      publishHTML([allowMissing: false,
                   alwaysLinkToLastBuild: false,
                   keepAll: true,
                   reportDir: 'build-coverage/coverage',
                   reportFiles: 'index.html',
                   reportName: 'Code Coverage'])
      publishHTML([allowMissing: false,
                   alwaysLinkToLastBuild: false,
                   keepAll: true,
                   reportDir: 'build-release/doc/html',
                   reportFiles: 'index.html',
                   reportName: 'API Documentation'])
    }
    currentBuild.result = 'SUCCESS'
  } catch (e) {
    currentBuild.result = 'FAILED'
    throw e;
  } finally {
    step([$class: 'StashNotifier'])
  }
}
