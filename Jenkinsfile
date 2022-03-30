pipeline {
  agent {
      node {
        label 'docker'
      }
  }
  triggers {
    pollSCM('H/5 * * * *')
  }
  options {
    parallelsAlwaysFailFast()
  }
  libraries {
      lib('fe-pipeline-steps@1.0.0')
  }
  environment {
    VERSION = feDetermineVersionFromGit()
    UNSTABLE = 'UNSTABLE'
  }
  stages {
    stage('Matrix') {
      matrix {
        agent {
          dockerfile {
            filename ".ci/Dockerfile.${env.DISTRO}"
            reuseNode true
          }
        }
        axes {
          axis {
            name 'DISTRO'
            values 'bionic', 'focal'
          }
        }
        stages {
          stage('Setup') {
            stages {
              stage('Notify Stash') {
                steps {
                  script {
                    notifyBitbucket()
                  }
                }
              }
              stage('Clean Workspace') {
                steps {
                  sh 'rm -rf build-*${DISTRO}'
                }
              }
            }
          }
          stage('Build') {
            stages {
              stage('Build debug') {
                steps {
                  dir("build-debug.${env.DISTRO}") {
                    sh '''
                      cmake -DCMAKE_BUILD_TYPE=Debug -DSTRICT=ON -DBUILD_COVERAGE=OFF \
                            -DBUILD_DOCUMENTATION=OFF -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON ..
                      make -j$(nproc)
                    '''
                  }
                }
              }
              stage('Build release') {
                steps {
                  dir("build-release.${env.DISTRO}") {
                    sh '''
                      cmake -DCMAKE_BUILD_TYPE=Release -DSTRICT=ON -DBUILD_COVERAGE=OFF \
                            -DBUILD_DOCUMENTATION=ON -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON ..
                      make -j$(nproc)
                    '''
                  }
                }
              }
              stage('Build examples (debug)') {
                steps {
                  dir("build-debug-examples.${env.DISTRO}") {
                    sh "cmake -DFranka_DIR:PATH=../build-debug.${DISTRO} ../examples"
                    sh 'make -j$(nproc)'
                  }
                }
              }
              stage('Build examples (release)') {
                steps {
                  dir("build-release-examples.${env.DISTRO}") {
                    sh "cmake -DFranka_DIR:PATH=../build-release.${DISTRO} ../examples"
                    sh 'make -j$(nproc)'
                  }
                }
              }
              stage('Build coverage') {
                steps {
                  dir("build-coverage.${env.DISTRO}") {
                    sh '''
                      cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_COVERAGE=ON \
                            -DBUILD_DOCUMENTATION=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=ON ..
                      make -j$(nproc)
                    '''
                  }
                }
              }
            }
          }
          stage('Analyze') {
            steps {
              dir("build-lint.${env.DISTRO}") {
                catchError(buildResult: env.UNSTABLE, stageResult: env.UNSTABLE) {
                  sh '''
                    cmake -DBUILD_COVERAGE=OFF -DBUILD_DOCUMENTATION=OFF -DBUILD_EXAMPLES=ON -DBUILD_TESTS=OFF ..
                    make check-format -j$(nproc)
                    make check-tidy -j$(nproc)
                  '''
                }
              }
              dir("build-coverage.${env.DISTRO}") {
                catchError(buildResult: env.UNSTABLE, stageResult: env.UNSTABLE) {
                  sh 'make coverage -j$(nproc)'
                  publishHTML([allowMissing: false,
                              alwaysLinkToLastBuild: false,
                              keepAll: true,
                              reportDir: 'coverage',
                              reportFiles: 'index.html',
                              reportName: "Code Coverage (${env.DISTRO})"])
                }
              }
            }
          }
          stage('Test') {
            steps {
              catchError(buildResult: env.UNSTABLE, stageResult: env.UNSTABLE) {
                dir("build-debug.${env.DISTRO}") {
                  sh 'ctest -V'
                }
                dir("build-release.${env.DISTRO}") {
                  sh 'ctest -V'
                }
              }
            }
            post {
              always {
                catchError(buildResult: env.UNSTABLE, stageResult: env.UNSTABLE) {
                  junit "build-release.${env.DISTRO}/test_results/*.xml"
                  junit "build-debug.${env.DISTRO}/test_results/*.xml"
                }
              }
            }
          }
          stage('Publish') {
            steps {
              dir("build-release.${env.DISTRO}") {
                catchError(buildResult: env.UNSTABLE, stageResult: env.UNSTABLE) {
                  sh 'cpack'
                  fePublishDebian('*.deb', 'futuretech-common', 'deb.distribution=bionic;deb.component=main;deb.architecture=amd64')
                  dir('doc') {
                    sh 'tar cfz ../libfranka-docs.tar.gz html'
                  }
                  sh "rename -e 's/(.tar.gz|.deb)\$/-${env.DISTRO}\$1/' *.deb *.tar.gz"
                  publishHTML([allowMissing: false,
                            alwaysLinkToLastBuild: false,
                            keepAll: true,
                            reportDir: 'doc/html',
                            reportFiles: 'index.html',
                            reportName: "API Documentation (${env.DISTRO})"])
                }
              }
            }
          }
        }
      }
    }
  }
  post {
    success {
      fePublishBuildInfo()
    }
    always {
      script {
        notifyBitbucket()
      }
    }
  }
}
