#!groovy

node {
  step([$class: 'StashNotifier'])

  try {
    checkout scm

    docker.build('libfranka-ci-worker', '.ci').inside {
      stage('Build (Debug)') {
        sh '.ci/debug.sh'
        junit 'build-debug/test_results/*.xml'
      }

      stage('Build (Release)') {
        sh '.ci/release.sh'
        archive 'build-release/*.deb, build-release/*.tar.gz'
      }

      stage('Build (Coverage)') {
        sh '.ci/coverage.sh'
        publishHTML([allowMissing: false,
                     alwaysLinkToLastBuild: false,
                     keepAll: true,
                     reportDir: 'build-coverage/coverage',
                     reportFiles: 'index.html',
                     reportName: 'Code Coverage'])
      }

      stage('Lint') {
        sh '.ci/lint.sh'
      }

      stage('Documentation') {
        sh '.ci/doc.sh'
        publishHTML([allowMissing: false,
                     alwaysLinkToLastBuild: false,
                     keepAll: true,
                     reportDir: 'build-doc/doc/html',
                     reportFiles: 'index.html',
                     reportName: 'API Documentation'])
      }
    }

    currentBuild.result = 'SUCCESS'
  } catch (e) {
    currentBuild.result = 'FAILED'
    throw e;
  } finally {
    step([$class: 'StashNotifier'])
  }
}
